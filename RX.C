
#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>
#include <U8g2lib.h>

// ==========================================
// CAN THRUSTER RX - FAST / AUTO RECOVERY
// UNO + MCP2515 + SH1107 OLED
// HL-52S V1.0 DUAL RELAY
// ==========================================

// ---------- PIN MAP ----------
const uint8_t CAN_CS_PIN = 10;
const uint8_t RELAY_LEFT_PIN = 4;
const uint8_t RELAY_RIGHT_PIN = 5;

// HL-52S LOW TRIGGER 기준
const uint8_t RELAY_ON = LOW;
const uint8_t RELAY_OFF = HIGH;

// ---------- CAN ----------
const uint16_t CMD_ID = 0x100;
const uint16_t ACK_ID = 0x101;

// TX와 같은 수정 발진자 설정
const CAN_CLOCK CAN_XTAL = MCP_8MHZ;

MCP2515 can(CAN_CS_PIN);

U8G2_SH1107_SEEED_128X128_2_HW_I2C oled(
  U8G2_R0, U8X8_PIN_NONE
);

// ---------- TIMING ----------
const unsigned long RX_TIMEOUT = 750;
const unsigned long OLED_INTERVAL = 250;
const unsigned long OLED_FAST_INTERVAL = 80;

const unsigned long RECOVERY_DELAY = 4000;
const unsigned long RETRY_INTERVAL = 4000;

// ---------- RECEIVED DATA ----------
int receivedADC = 512;
int receivedPercent = 0;

bool leftPressed = false;
bool rightPressed = false;

bool relayLeft = false;
bool relayRight = false;

bool canReady = false;
bool everReceived = false;
bool uiDirty = true;

uint8_t receivedSequence = 0;

// ---------- TIMER ----------
unsigned long lastRX = 0;
unsigned long lastOLED = 0;
unsigned long lastRecovery = 0;

// ---------- COUNTERS ----------
unsigned long rxOK = 0;
unsigned long ackOK = 0;
unsigned long ackFail = 0;
unsigned long relayChanges = 0;
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
// LINK STATUS
// ==========================================
bool linkOK() {
  return canReady &&
         everReceived &&
         millis() - lastRX <= RX_TIMEOUT;
}

// ==========================================
// RELAY CONTROL
// 상태가 변경될 때만 출력
// ==========================================
void setRelay(bool newLeft, bool newRight) {

  // 좌우 동시 ON 금지
  if (newLeft && newRight) {
    newLeft = false;
    newRight = false;
  }

  if (newLeft == relayLeft &&
      newRight == relayRight) {
    return;
  }

  // 먼저 OFF할 릴레이 처리
  if (relayLeft && !newLeft) {
    digitalWrite(RELAY_LEFT_PIN, RELAY_OFF);
  }

  if (relayRight && !newRight) {
    digitalWrite(RELAY_RIGHT_PIN, RELAY_OFF);
  }

  // 그다음 필요한 릴레이만 ON
  if (newLeft && !relayLeft) {
    digitalWrite(RELAY_LEFT_PIN, RELAY_ON);
  }

  if (newRight && !relayRight) {
    digitalWrite(RELAY_RIGHT_PIN, RELAY_ON);
  }

  relayLeft = newLeft;
  relayRight = newRight;

  relayChanges++;
  uiDirty = true;

  Serial.print(F("RELAY L="));
  Serial.print(relayLeft ? "ON" : "OFF");

  Serial.print(F(" R="));
  Serial.println(relayRight ? "ON" : "OFF");
}

// ==========================================
// CAN INITIALIZATION
// ==========================================
bool initCAN() {
  can.reset();

  if (can.setBitrate(CAN_500KBPS, CAN_XTAL)
      != MCP2515::ERROR_OK) {
    Serial.println(F("RX BITRATE FAIL"));
    return false;
  }

  if (can.setNormalMode()
      != MCP2515::ERROR_OK) {
    Serial.println(F("RX NORMAL FAIL"));
    return false;
  }

  resetCount++;
  Serial.println(F("RX CAN INITIALIZED"));

  return true;
}

// ==========================================
// SEND ACK
// ==========================================
void sendACK(uint8_t seq) {
  if (!canReady) return;

  struct can_frame frame;

  frame.can_id = ACK_ID;
  frame.can_dlc = 3;

  frame.data[0] = 0xA5;
  frame.data[1] = seq;
  frame.data[2] = checksum(frame.data, 2);

  if (can.sendMessage(&frame)
      == MCP2515::ERROR_OK) {
    ackOK++;
  } else {
    ackFail++;
  }
}

// ==========================================
// RECEIVE COMMAND
// ==========================================
void receiveCAN() {
  if (!canReady) return;

  struct can_frame frame;

  // 최대 16개 대기 메시지 처리
  for (uint8_t i = 0; i < 16; i++) {

    if (can.readMessage(&frame)
        != MCP2515::ERROR_OK) {
      break;
    }

    if (frame.can_id != CMD_ID ||
        frame.can_dlc != 7) {
      continue;
    }

    if (frame.data[5] != 1) {
      continue;
    }

    if (frame.data[6] !=
        checksum(frame.data, 6)) {
      continue;
    }

    uint16_t adc =
      (uint16_t)frame.data[1] |
      ((uint16_t)frame.data[2] << 8);

    if (adc > 1023) continue;

    uint8_t flags = frame.data[3];

    if (flags & 0xFC) continue;

    int percent = (int8_t)frame.data[4];

    if (percent < -100 || percent > 100) {
      continue;
    }

    // ---------- VALID FRAME ----------
    int oldPercent = receivedPercent;

    bool oldLeft = leftPressed;
    bool oldRight = rightPressed;

    receivedADC = adc;
    receivedPercent = percent;

    leftPressed = (flags & 0x01) != 0;
    rightPressed = (flags & 0x02) != 0;

    receivedSequence = frame.data[0];

    lastRX = millis();
    everReceived = true;
    rxOK++;

    // 스위치 신호 즉시 릴레이 적용
    setRelay(leftPressed, rightPressed);

    // 화면 표시 변경 예약
    if (oldPercent != receivedPercent ||
        oldLeft != leftPressed ||
        oldRight != rightPressed) {
      uiDirty = true;
    }

    // 정상 수신 ACK
    sendACK(receivedSequence);
  }
}

// ==========================================
// LINK WATCHDOG
// ==========================================
void checkConnection() {
  if (linkOK()) return;

  // 명령을 더 이상 받지 못하면 OFF
  setRelay(false, false);
}

// ==========================================
// AUTOMATIC RECOVERY
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

    // 수신이 장시간 중단된 경우
    if (now - lastRX > RECOVERY_DELAY) {
      mustRecover = true;
    }
  }

  if (!mustRecover) return;

  lastRecovery = now;

  // 복구 전 출력 OFF
  setRelay(false, false);

  canReady = false;
  everReceived = false;

  Serial.println(F("RX CAN RECOVERING"));

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

    oled.drawStr(3, 12, "CAN RX / RELAY");
    oled.drawHLine(3, 16, 122);

    oled.setFont(u8g2_font_helvB18_tr);

    snprintf(s, sizeof(s),
             "%+d%%", receivedPercent);

    oled.drawStr(7, 43, s);

    oled.setFont(u8g2_font_6x12_tr);

    if (receivedPercent < 0)
      oled.drawStr(4, 57, "DIR: PORT");
    else if (receivedPercent > 0)
      oled.drawStr(4, 57, "DIR: STARBOARD");
    else
      oled.drawStr(4, 57, "DIR: NEUTRAL");

    snprintf(s, sizeof(s),
             "LEFT SW : %s",
             leftPressed ? "ON" : "OFF");
    oled.drawStr(4, 71, s);

    snprintf(s, sizeof(s),
             "RIGHT SW: %s",
             rightPressed ? "ON" : "OFF");
    oled.drawStr(4, 84, s);

    snprintf(s, sizeof(s),
             "RELAY L:%s R:%s",
             relayLeft ? "ON" : "OFF",
             relayRight ? "ON" : "OFF");
    oled.drawStr(4, 97, s);

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

  Serial.print(F("RX OK="));
  Serial.print(rxOK);

  Serial.print(F(" ACK_OK="));
  Serial.print(ackOK);

  Serial.print(F(" ACK_FAIL="));
  Serial.print(ackFail);

  Serial.print(F(" RELAY_CHG="));
  Serial.print(relayChanges);

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

  // 릴레이 초기 상태 OFF
  digitalWrite(RELAY_LEFT_PIN, RELAY_OFF);
  digitalWrite(RELAY_RIGHT_PIN, RELAY_OFF);

  pinMode(RELAY_LEFT_PIN, OUTPUT);
  pinMode(RELAY_RIGHT_PIN, OUTPUT);

  relayLeft = false;
  relayRight = false;

  Wire.begin();

  oled.begin();

  // OLED I2C 400kHz
  oled.setBusClock(400000);

  SPI.begin();

  canReady = initCAN();

  lastRecovery = millis();

  Serial.println(F("RX READY"));
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long now = millis();

  // 1. CAN 명령 수신 최우선
  receiveCAN();

  // 2. 통신 단절 시 출력 차단
  checkConnection();

  // 3. 통신 복구
  recoverCAN();

  // 4. OLED 후순위 갱신
  now = millis();

  unsigned long drawInterval =
    uiDirty ? OLED_FAST_INTERVAL :
              OLED_INTERVAL;

  if (now - lastOLED >= drawInterval) {
    lastOLED = now;
    uiDirty = false;
    drawOLED();
  }

  // 5. 진단 로그
  printDiagnostics();
}
