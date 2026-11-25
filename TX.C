
#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>
#include <U8g2lib.h>

// ==========================================
// CAN THRUSTER TX - FAST / AUTO RECOVERY
// UNO + B10K + 2 NO SWITCH + SH1107
// MCP2515 / 500kbps / ID 0x100
// ==========================================

// ---------- PIN MAP ----------
const uint8_t POT_PIN = A0;
const uint8_t LEFT_SW_PIN = 4;
const uint8_t RIGHT_SW_PIN = 5;
const uint8_t CAN_CS_PIN = 10;

// ---------- CAN ----------
const uint16_t CMD_ID = 0x100;
const uint16_t ACK_ID = 0x101;

// MCP2515 수정 발진자가 16MHz이면 변경
const CAN_CLOCK CAN_XTAL = MCP_8MHZ;

MCP2515 can(CAN_CS_PIN);

// 2페이지 버퍼 - UNO 메모리 절약 + 속도 개선
U8G2_SH1107_SEEED_128X128_2_HW_I2C oled(
  U8G2_R0, U8X8_PIN_NONE
);

// ---------- TIMING ----------
const unsigned long TX_INTERVAL = 40;
const unsigned long POT_INTERVAL = 5;
const unsigned long OLED_INTERVAL = 250;
const unsigned long OLED_FAST_INTERVAL = 80;

const unsigned long DEBOUNCE_MS = 12;
const unsigned long ACK_TIMEOUT = 1500;
const unsigned long RECOVERY_DELAY = 3500;
const unsigned long RETRY_INTERVAL = 4000;

// ---------- POT FILTER ----------
int32_t filteredPotQ8 = 0;
int stableADC = 512;
int potPercent = 0;

const int ADC_HYSTERESIS = 5;

// ---------- SWITCH STATE ----------
bool leftRaw = false;
bool rightRaw = false;

bool leftPressed = false;
bool rightPressed = false;

unsigned long leftChangedAt = 0;
unsigned long rightChangedAt = 0;

// ---------- CAN STATE ----------
bool canReady = false;
bool receivedAck = false;
bool uiDirty = true;

uint8_t sequence = 0;

unsigned long lastTX = 0;
unsigned long lastPOT = 0;
unsigned long lastOLED = 0;
unsigned long lastACK = 0;
unsigned long lastRecovery = 0;

unsigned long txOK = 0;
unsigned long txFail = 0;
unsigned long ackOK = 0;
unsigned long resetCount = 0;

// ==========================================
// CHECKSUM
// ==========================================
uint8_t checksum(const uint8_t *data,
                 uint8_t length) {
  uint8_t result = 0x5A;

  for (uint8_t i = 0; i < length; i++) {
    result ^= data[i];
  }

  return result;
}

// ==========================================
// CAN INITIALIZATION
// ==========================================
bool initCAN() {
  can.reset();

  if (can.setBitrate(CAN_500KBPS, CAN_XTAL)
      != MCP2515::ERROR_OK) {
    Serial.println(F("TX BITRATE FAIL"));
    return false;
  }

  if (can.setNormalMode()
      != MCP2515::ERROR_OK) {
    Serial.println(F("TX NORMAL FAIL"));
    return false;
  }

  resetCount++;
  Serial.println(F("TX CAN INITIALIZED"));

  return true;
}

// ==========================================
// LINK STATUS
// ==========================================
bool linkOK() {
  return canReady &&
         receivedAck &&
         millis() - lastACK <= ACK_TIMEOUT;
}

// ==========================================
// POTENTIOMETER FILTER
// ==========================================
void updatePot() {
  unsigned long now = millis();

  if (now - lastPOT < POT_INTERVAL) return;
  lastPOT = now;

  int sum = 0;

  for (uint8_t i = 0; i < 4; i++) {
    sum += analogRead(POT_PIN);
  }

  int raw = (sum + 2) / 4;

  // 소수부를 유지하는 IIR 필터
  int32_t target = (int32_t)raw * 256L;
  filteredPotQ8 +=
    (target - filteredPotQ8) / 4;

  int filtered =
    (int)((filteredPotQ8 + 128L) / 256L);

  // 작은 ADC 잡음 무시
  if (abs(filtered - stableADC)
      >= ADC_HYSTERESIS) {
    stableADC = filtered;
  }

  // ADC 0~1023 -> -100~+100
  long scaled =
    ((long)stableADC * 200L + 511L) / 1023L;

  int nextPercent =
    constrain((int)scaled - 100, -100, 100);

  if (nextPercent != potPercent) {
    potPercent = nextPercent;
    uiDirty = true;
  }
}

// ==========================================
// LIMIT SWITCH DEBOUNCE
// INPUT_PULLUP: LOW = PRESSED
// ==========================================
bool updateSwitches() {
  unsigned long now = millis();
  bool changed = false;

  bool currentLeft =
    digitalRead(LEFT_SW_PIN) == LOW;

  bool currentRight =
    digitalRead(RIGHT_SW_PIN) == LOW;

  if (currentLeft != leftRaw) {
    leftRaw = currentLeft;
    leftChangedAt = now;
  }

  if (currentRight != rightRaw) {
    rightRaw = currentRight;
    rightChangedAt = now;
  }

  if (now - leftChangedAt >= DEBOUNCE_MS &&
      leftPressed != leftRaw) {
    leftPressed = leftRaw;
    changed = true;
  }

  if (now - rightChangedAt >= DEBOUNCE_MS &&
      rightPressed != rightRaw) {
    rightPressed = rightRaw;
    changed = true;
  }

  if (changed) {
    uiDirty = true;

    Serial.print(F("SW LEFT="));
    Serial.print(leftPressed);
    Serial.print(F(" RIGHT="));
    Serial.println(rightPressed);
  }

  return changed;
}

// ==========================================
// RECEIVE CAN ACK (0x101)
// ==========================================
void receiveACK() {
  if (!canReady) return;

  struct can_frame frame;

  for (uint8_t i = 0; i < 16; i++) {

    if (can.readMessage(&frame)
        != MCP2515::ERROR_OK) {
      break;
    }

    if (frame.can_id != ACK_ID ||
        frame.can_dlc != 3) {
      continue;
    }

    if (frame.data[0] != 0xA5) continue;

    if (frame.data[2] !=
        checksum(frame.data, 2)) {
      continue;
    }

    lastACK = millis();
    receivedAck = true;
    ackOK++;
  }
}

// ==========================================
// TRANSMIT CAN COMMAND (0x100)
// ==========================================
void sendCAN() {
  if (!canReady) return;

  struct can_frame frame;

  uint16_t adc = (uint16_t)stableADC;
  uint8_t flags = 0;

  if (leftPressed)  flags |= 0x01;
  if (rightPressed) flags |= 0x02;

  frame.can_id = CMD_ID;
  frame.can_dlc = 7;

  frame.data[0] = sequence++;
  frame.data[1] = lowByte(adc);
  frame.data[2] = highByte(adc);
  frame.data[3] = flags;
  frame.data[4] = (uint8_t)(int8_t)potPercent;
  frame.data[5] = 1;
  frame.data[6] = checksum(frame.data, 6);

  if (can.sendMessage(&frame)
      == MCP2515::ERROR_OK) {
    txOK++;
  } else {
    txFail++;
  }
}

// ==========================================
// AUTOMATIC CAN RECOVERY
// ==========================================
void recoverCAN() {
  unsigned long now = millis();

  if (now - lastRecovery < RETRY_INTERVAL) {
    return;
  }

  bool mustRecover = !canReady;

  if (canReady) {
    uint8_t eflg = can.getErrorFlags();

    // TX Bus-Off
    if (eflg & 0x20) {
      mustRecover = true;
    }

    // ACK가 장시간 없는 경우 복구
    if (now - lastACK > RECOVERY_DELAY) {
      mustRecover = true;
    }
  }

  if (!mustRecover) return;

  lastRecovery = now;

  Serial.println(F("TX CAN RECOVERING"));

  canReady = false;
  receivedAck = false;

  canReady = initCAN();
  uiDirty = true;
}

// ==========================================
// OLED
// ==========================================
void drawOLED() {
  char s[24];

  oled.firstPage();

  do {
    oled.setFont(u8g2_font_6x12_tr);

    oled.drawStr(3, 12, "CAN TX / CONTROL");
    oled.drawHLine(3, 16, 122);

    oled.setFont(u8g2_font_helvB18_tr);

    snprintf(s, sizeof(s),
             "%+d%%", potPercent);

    oled.drawStr(7, 43, s);

    oled.setFont(u8g2_font_6x12_tr);

    if (potPercent < 0)
      oled.drawStr(4, 57, "DIR: PORT");
    else if (potPercent > 0)
      oled.drawStr(4, 57, "DIR: STARBOARD");
    else
      oled.drawStr(4, 57, "DIR: NEUTRAL");

    snprintf(s, sizeof(s),
             "LEFT SW : %s",
             leftPressed ? "ON" : "OFF");
    oled.drawStr(4, 72, s);

    snprintf(s, sizeof(s),
             "RIGHT SW: %s",
             rightPressed ? "ON" : "OFF");
    oled.drawStr(4, 85, s);

    oled.drawStr(4, 99,
                 "ID:0x100  500kbps");

    oled.drawHLine(3, 104, 122);

    if (!canReady)
      oled.drawStr(4, 117, "CAN INIT FAIL");
    else if (linkOK())
      oled.drawStr(4, 117, "LINK OK");
    else
      oled.drawStr(4, 117, "DISCONNECTED");

  } while (oled.nextPage());
}

// ==========================================
// SERIAL DIAGNOSTICS
// ==========================================
void printDiagnostics() {
  static unsigned long lastPrint = 0;

  unsigned long now = millis();

  if (now - lastPrint < 1000) return;
  lastPrint = now;

  Serial.print(F("TX OK="));
  Serial.print(txOK);

  Serial.print(F(" FAIL="));
  Serial.print(txFail);

  Serial.print(F(" ACK="));
  Serial.print(ackOK);

  Serial.print(F(" RESET="));
  Serial.print(resetCount);

  Serial.print(F(" LINK="));
  Serial.print(linkOK() ? "OK" : "LOST");

  Serial.print(F(" EFLG=0x"));

  if (canReady)
    Serial.print(can.getErrorFlags(), HEX);
  else
    Serial.print(F("NA"));

  Serial.println();
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_SW_PIN, INPUT_PULLUP);
  pinMode(RIGHT_SW_PIN, INPUT_PULLUP);

  leftRaw =
    digitalRead(LEFT_SW_PIN) == LOW;

  rightRaw =
    digitalRead(RIGHT_SW_PIN) == LOW;

  leftPressed = leftRaw;
  rightPressed = rightRaw;

  int initialADC = analogRead(POT_PIN);

  filteredPotQ8 =
    (int32_t)initialADC * 256L;

  stableADC = initialADC;

  long scaled =
    ((long)stableADC * 200L + 511L) / 1023L;

  potPercent =
    constrain((int)scaled - 100, -100, 100);

  Wire.begin();

  oled.begin();

  // OLED I2C 400kHz
  oled.setBusClock(400000);

  SPI.begin();

  canReady = initCAN();

  lastRecovery = millis();

  Serial.println(F("TX READY"));
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long now = millis();

  // 1. 입력 우선
  updatePot();
  bool switchChanged = updateSwitches();

  // 2. ACK 수신 우선
  receiveACK();

  // 3. 정기 전송 + 스위치 변경 즉시 전송
  if (switchChanged ||
      now - lastTX >= TX_INTERVAL) {
    lastTX = millis();
    sendCAN();
  }

  // 4. 재초기화
  recoverCAN();

  // 5. OLED는 후순위
  now = millis();

  unsigned long drawInterval =
    uiDirty ? OLED_FAST_INTERVAL :
              OLED_INTERVAL;

  if (now - lastOLED >= drawInterval) {
    lastOLED = now;
    uiDirty = false;
    drawOLED();
  }

  // 6. 진단 로그
  printDiagnostics();
}
