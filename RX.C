
/*
  CAN MONITOR V4 - UNO #2 RECEIVER

  OLED : SDA A4, SCL A5
  CAN  : CS D10, SI D11, SO D12,
         SCK D13, INT D2

  0x100 : 조작값 수신
  0x101 : 검증 응답 송신

  LED 없음 / 화면 점멸 없음
*/

#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>
#include <U8g2lib.h>

// ===== 기본 설정 =====
const byte CAN_CS = 10;

const uint16_t RX_ID = 0x100;
const uint16_t ACK_ID = 0x101;

// 실제 크리스털이 16MHz라면 변경
const CAN_CLOCK CAN_OSC = MCP_8MHZ;

const unsigned long UI_INTERVAL = 180;
const unsigned long LINK_TIMEOUT = 1200;
const unsigned long RECOVERY_INTERVAL = 4000;

MCP2515 can(CAN_CS, 8000000);

U8G2_SH1107_SEEED_128X128_1_HW_I2C oled(
  U8G2_R0, U8X8_PIN_NONE
);

// ===== 상태 변수 =====
int receivedPercent = 0;

byte receivedSequence = 0;
byte lastData[5] = {0};

unsigned long lastRx = 0;
unsigned long lastUi = 0;
unsigned long lastRecovery = 0;
unsigned long lastDiag = 0;
unsigned long linkStarted = 0;

bool canReady = false;
bool receivedOnce = false;
bool wasOnline = false;

// ===== 방향 표시 =====
const char* directionText(int value) {

  if (value < 0) return "PORT";
  if (value > 0) return "STARBOARD";

  return "NEUTRAL";
}

// ===== XOR 검증값 계산 =====
byte checksum(const byte* data) {

  byte result = 0xA5;

  for (byte i = 0; i < 4; i++) {
    result ^= data[i];
  }

  return result;
}

// ===== CAN 초기화 =====
bool initCAN() {

  can.reset();

  MCP2515::ERROR result =
    can.setBitrate(CAN_500KBPS, CAN_OSC);

  if (result == MCP2515::ERROR_OK) {
    result = can.setNormalMode();
  }

  canReady = (result == MCP2515::ERROR_OK);

  Serial.print(F("CAN INIT: "));
  Serial.println(
    canReady ? F("OK") : F("FAIL")
  );

  return canReady;
}

// ===== 통신 상태 확인 =====
bool linkOnline() {

  return canReady &&
         receivedOnce &&
         (millis() - lastRx < LINK_TIMEOUT);
}

// ===== 상태 문구 =====
const char* linkText() {

  unsigned long now = millis();

  if (linkOnline()) {

    if (now - linkStarted < 900) {
      return "RECOVERING";
    }

    return "LINK OK";
  }

  if (now - lastRecovery < 1200) {
    return "RECOVERING";
  }

  return "DISCONNECTED";
}

// ===== 정상 CAN 데이터 수신 =====
void receiveCAN() {

  struct can_frame frame = {};

  // 한 번에 최대 6개 메시지를 읽는다.
  for (byte n = 0; n < 6; n++) {

    if (can.readMessage(&frame)
        != MCP2515::ERROR_OK) {
      break;
    }

    // 표준 CAN 데이터 프레임만 사용
    if (frame.can_id != RX_ID ||
        frame.can_dlc != 5) {
      continue;
    }

    // 데이터 검증 실패 시 무시
    if (checksum(frame.data) != frame.data[4]) {
      Serial.println(F("CHECKSUM ERROR"));
      continue;
    }

    // 두 바이트를 16비트 값으로 복원
    uint16_t raw =
      (uint16_t)frame.data[1] |
      ((uint16_t)frame.data[2] << 8);

    // 음수까지 복원
    int32_t value = raw;

    if (raw >= 0x8000) {
      value -= 65536L;
    }

    // 조작값 범위 검사
    if (value < -100 || value > 100) {
      Serial.println(F("RANGE ERROR"));
      continue;
    }

    // 방향 코드 검사
    byte expectedDirection = 0;

    if (value < 0) expectedDirection = 1;
    if (value > 0) expectedDirection = 2;

    if (frame.data[3] != expectedDirection) {
      Serial.println(F("DIRECTION ERROR"));
      continue;
    }

    // 정상 메시지 수신
    receivedPercent = (int)value;
    receivedSequence = frame.data[0];

    for (byte i = 0; i < 5; i++) {
      lastData[i] = frame.data[i];
    }

    lastRx = millis();
    receivedOnce = true;

    // ===== 송신측으로 응답 =====

    struct can_frame reply = {};

    reply.can_id = ACK_ID;
    reply.can_dlc = 5;

    // 수신한 데이터 5바이트를 그대로 반환
    for (byte i = 0; i < 5; i++) {
      reply.data[i] = frame.data[i];
    }

    MCP2515::ERROR result =
      can.sendMessage(&reply);

    if (result != MCP2515::ERROR_OK) {
      Serial.print(F("ECHO ERROR: "));
      Serial.println((int)result);
    }
  }
}

// ===== 자동 복구 =====
void checkRecovery() {

  unsigned long now = millis();

  bool online = linkOnline();

  if (online && !wasOnline) {
    linkStarted = now;
    Serial.println(F("LINK RESTORED"));
  }

  if (!online && wasOnline) {
    Serial.println(F("LINK LOST"));
  }

  wasOnline = online;

  if (online) return;

  // 4초마다 재초기화
  if (now - lastRecovery >= RECOVERY_INTERVAL) {

    lastRecovery = now;

    Serial.println(F("CAN RECOVERY"));

    canReady = initCAN();

    receivedOnce = false;
  }
}

// ===== HEX 문자열 생성 =====
void formatHex(char* output, size_t size) {

  snprintf(
    output, size,
    "%02X %02X %02X %02X %02X",
    lastData[0], lastData[1], lastData[2],
    lastData[3], lastData[4]
  );
}

// ===== OLED 표시 =====
void drawOLED() {

  char percentStr[12];
  char infoStr[28];
  char hexStr[20];

  bool online = linkOnline();

  if (online) {
    snprintf(
      percentStr, sizeof(percentStr),
      "%+d%%", receivedPercent
    );
  } else {
    strcpy(percentStr, "--");
  }

  formatHex(hexStr, sizeof(hexStr));

  oled.firstPage();

  do {

    oled.setFont(u8g2_font_6x10_tf);

    oled.drawStr(2, 10, "CAN / RX MONITOR");
    oled.drawHLine(0, 14, 128);

    // 수신 조작값
    oled.setFont(u8g2_font_helvB18_tf);
    oled.drawStr(2, 39, percentStr);

    oled.setFont(u8g2_font_6x10_tf);

    oled.drawStr(
      2, 52,
      online ? directionText(receivedPercent)
             : "NO SIGNAL"
    );

    oled.drawStr(2, 67, "ID:0x100  500kbps");

    snprintf(
      infoStr, sizeof(infoStr),
      "DLC:5 SEQ:%03u",
      receivedSequence
    );

    oled.drawStr(2, 79, infoStr);

    // 실제 수신한 CAN 데이터 HEX
    oled.setFont(u8g2_font_5x8_tf);

    if (online) {
      oled.drawStr(2, 91, hexStr);
    } else {
      oled.drawStr(2, 91, "-- -- -- -- --");
    }

    // 마지막 수신 경과시간
    oled.setFont(u8g2_font_6x10_tf);

    if (online) {

      snprintf(
        infoStr, sizeof(infoStr),
        "AGE:%lu ms",
        millis() - lastRx
      );

    } else {
      strcpy(infoStr, "AGE: --");
    }

    oled.drawStr(2, 106, infoStr);

    oled.drawHLine(0, 111, 128);

    // 정상 / 단절 / 복구 상태
    oled.drawStr(2, 124, linkText());

  } while (oled.nextPage());
}

// ===== 최초 한 번 실행 =====
void setup() {

  Serial.begin(115200);

  pinMode(CAN_CS, OUTPUT);
  digitalWrite(CAN_CS, HIGH);

  Wire.begin();
  oled.begin();

  SPI.begin();

  lastRecovery = millis();

  initCAN();
  drawOLED();
}

// ===== 계속 반복 =====
void loop() {

  unsigned long now = millis();

  if (canReady) {

    // CAN 조작값 수신
    receiveCAN();
  }

  // 통신 상태 및 자동 복구
  checkRecovery();

  // OLED 갱신
  if (now - lastUi >= UI_INTERVAL) {

    lastUi = now;
    drawOLED();
  }

  // 2초마다 시리얼 진단 출력
  if (now - lastDiag >= 2000) {

    lastDiag = now;

    Serial.print(F("SEQ="));
    Serial.print(receivedSequence);

    Serial.print(F(" TEC="));
    Serial.print(can.errorCountTX());

    Serial.print(F(" REC="));
    Serial.println(can.errorCountRX());
  }
}
