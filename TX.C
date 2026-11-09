
/*
  CAN MONITOR V4 - UNO #1 SENDER

  B10K : 가운데 A0, 양쪽 5V/GND
  OLED : SDA A4, SCL A5
  CAN  : CS D10, SI D11, SO D12,
         SCK D13, INT D2

  0x100 : 조작값 송신
  0x101 : 수신측 응답 수신

  DATA 5바이트 구성
  [0] 메시지 순번 SEQ
  [1] 조작값 하위 바이트
  [2] 조작값 상위 바이트
  [3] 방향 (0=중립, 1=좌현, 2=우현)
  [4] 검증값 XOR

  LED 없음 / 화면 점멸 없음
  포텐셔미터 강제 보정 없음
*/

#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>
#include <U8g2lib.h>

// ===== 기본 설정 =====
const byte POT_PIN = A0;
const byte CAN_CS = 10;

const uint16_t TX_ID = 0x100;
const uint16_t ACK_ID = 0x101;

// 실제 CAN 모듈 발진자 확인
const CAN_CLOCK CAN_OSC = MCP_8MHZ;

// 송신 120ms, 화면 갱신 180ms
const unsigned long TX_INTERVAL = 120;
const unsigned long UI_INTERVAL = 180;

// 응답이 1.2초 없으면 단절 표시
const unsigned long LINK_TIMEOUT = 1200;

// 4초 이상 통신이 끊기면 복구 시도
const unsigned long RECOVERY_INTERVAL = 4000;

MCP2515 can(CAN_CS, 8000000);

// SH1107 128x128, 페이지 버퍼
U8G2_SH1107_SEEED_128X128_1_HW_I2C oled(
  U8G2_R0, U8X8_PIN_NONE
);

// ===== 변수 =====
int adcValue = 0;
int potPercent = 0;

byte sequence = 0;
byte lastData[5] = {0};

unsigned long lastTx = 0;
unsigned long lastUi = 0;
unsigned long lastAck = 0;
unsigned long lastRecovery = 0;
unsigned long lastDiag = 0;
unsigned long lastSendTime = 0;
unsigned long linkStarted = 0;

unsigned int rttMs = 0;

bool canReady = false;
bool receivedAck = false;
bool wasOnline = false;

// ===== 포텐셔미터 측정 =====
void readPot() {

  // 4번 측정하여 평균을 구한다.
  // 강제 끝단 보정 및 데드존은 없다.
  long sum = 0;

  for (byte i = 0; i < 4; i++) {
    sum += analogRead(POT_PIN);
  }

  adcValue = sum / 4;

  // ADC 0~1023을 -100~+100%로 변환
  potPercent = map(
    adcValue, 0, 1023, -100, 100
  );
}

// ===== 조작 방향 =====
byte directionCode(int value) {

  if (value < 0) return 1; // PORT
  if (value > 0) return 2; // STARBOARD

  return 0;                // NEUTRAL
}

const char* directionText(int value) {

  if (value < 0) return "PORT";
  if (value > 0) return "STARBOARD";

  return "NEUTRAL";
}

// ===== 데이터 검증용 XOR =====
// 앞의 4바이트를 XOR하여 검증값 생성
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

  // CAN 속도: 500kbps
  MCP2515::ERROR result =
    can.setBitrate(CAN_500KBPS, CAN_OSC);

  if (result == MCP2515::ERROR_OK) {

    // 일반 CAN 통신 모드
    result = can.setNormalMode();
  }

  canReady = (result == MCP2515::ERROR_OK);

  Serial.print(F("CAN INIT: "));
  Serial.println(
    canReady ? F("OK") : F("FAIL")
  );

  return canReady;
}

// ===== 정상 통신 여부 =====
bool linkOnline() {

  return canReady &&
         receivedAck &&
         (millis() - lastAck < LINK_TIMEOUT);
}

// ===== 통신 상태 문구 =====
const char* linkText() {

  unsigned long now = millis();

  if (linkOnline()) {

    // 연결이 복구된 직후 잠깐 표시
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

// ===== CAN 송신 =====
void sendValue() {

  struct can_frame frame = {};

  frame.can_id = TX_ID;
  frame.can_dlc = 5;

  // 순번을 매번 1 증가시킨다.
  sequence++;
  frame.data[0] = sequence;

  // 부호 있는 조작값을 16비트로 저장
  uint16_t raw =
    (uint16_t)((int16_t)potPercent);

  frame.data[1] = raw & 0xFF;
  frame.data[2] = (raw >> 8) & 0xFF;

  // 좌현/중립/우현
  frame.data[3] = directionCode(potPercent);

  // 데이터 검증값
  frame.data[4] = checksum(frame.data);

  // 가장 최근 송신값을 저장
  // 나중에 수신측 응답과 비교한다.
  for (byte i = 0; i < 5; i++) {
    lastData[i] = frame.data[i];
  }

  lastSendTime = millis();

  MCP2515::ERROR result =
    can.sendMessage(&frame);

  // 오류 결과를 PC 시리얼 모니터에 출력
  if (result != MCP2515::ERROR_OK) {
    Serial.print(F("TX ERROR: "));
    Serial.println((int)result);
  }
}

// ===== 수신측 응답 확인 =====
void readAck() {

  struct can_frame frame = {};

  // 대기 중인 CAN 메시지를 최대 6개 처리
  for (byte n = 0; n < 6; n++) {

    if (can.readMessage(&frame)
        != MCP2515::ERROR_OK) {
      break;
    }

    // 표준 데이터 프레임만 허용
    if (frame.can_id != ACK_ID ||
        frame.can_dlc != 5) {
      continue;
    }

    // 체크섬 검사
    if (checksum(frame.data) != frame.data[4]) {
      continue;
    }

    // 가장 최근 송신한 5바이트와 비교
    bool match = true;

    for (byte i = 0; i < 5; i++) {

      if (frame.data[i] != lastData[i]) {
        match = false;
        break;
      }
    }

    if (!match) continue;

    unsigned long now = millis();

    // 왕복 응답시간 RTT
    rttMs = (unsigned int)(now - lastSendTime);

    lastAck = now;
    receivedAck = true;
  }
}

// ===== 자동 복구 =====
void checkRecovery() {

  unsigned long now = millis();

  bool online = linkOnline();

  // 통신이 처음 복구된 시각 저장
  if (online && !wasOnline) {
    linkStarted = now;
    Serial.println(F("LINK RESTORED"));
  }

  if (!online && wasOnline) {
    Serial.println(F("LINK LOST"));
  }

  wasOnline = online;

  // 통신이 정상이라면 초기화하지 않는다.
  if (online) return;

  // 일정 간격마다 복구 시도
  if (now - lastRecovery >= RECOVERY_INTERVAL) {

    lastRecovery = now;

    Serial.println(F("CAN RECOVERY"));

    canReady = initCAN();

    // 재초기화 후 응답을 다시 확인한다.
    receivedAck = false;
  }
}

// ===== 16진수 문자 생성 =====
void formatHex(char* output, size_t size) {

  snprintf(
    output, size,
    "%02X %02X %02X %02X %02X",
    lastData[0], lastData[1], lastData[2],
    lastData[3], lastData[4]
  );
}

// ===== OLED 화면 출력 =====
void drawOLED() {

  char percentStr[12];
  char infoStr[28];
  char hexStr[20];

  snprintf(
    percentStr, sizeof(percentStr),
    "%+d%%", potPercent
  );

  formatHex(hexStr, sizeof(hexStr));

  oled.firstPage();

  do {

    oled.setFont(u8g2_font_6x10_tf);

    oled.drawStr(2, 10, "CAN / TX MONITOR");
    oled.drawHLine(0, 14, 128);

    // 포텐셔미터 위치
    oled.setFont(u8g2_font_helvB18_tf);
    oled.drawStr(2, 39, percentStr);

    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(
      2, 52, directionText(potPercent)
    );

    // CAN 통신 정보
    oled.drawStr(2, 67, "ID:0x100  500kbps");

    snprintf(
      infoStr, sizeof(infoStr),
      "DLC:5 SEQ:%03u", sequence
    );
    oled.drawStr(2, 79, infoStr);

    // 실제 5바이트 데이터
    oled.setFont(u8g2_font_5x8_tf);
    oled.drawStr(2, 91, hexStr);

    // RTT
    oled.setFont(u8g2_font_6x10_tf);
    if (linkOnline()) {

      snprintf(
        infoStr, sizeof(infoStr),
        "RTT:%u ms", rttMs
      );

    } else {
      strcpy(infoStr, "RTT: --");
    }

    oled.drawStr(2, 106, infoStr);

    oled.drawHLine(0, 111, 128);

    // 점멸하지 않는 통신 상태
    oled.drawStr(2, 124, linkText());

  } while (oled.nextPage());
}

// ===== 최초 1회 실행 =====
void setup() {

  Serial.begin(115200);

  pinMode(POT_PIN, INPUT);

  pinMode(CAN_CS, OUTPUT);
  digitalWrite(CAN_CS, HIGH);

  Wire.begin();
  oled.begin();

  SPI.begin();

  lastRecovery = millis();

  initCAN();

  readPot();
  drawOLED();
}

// ===== 계속 반복 =====
void loop() {

  unsigned long now = millis();

  readPot();

  // CAN 수신 응답 확인
  if (canReady) {
    readAck();

    // 120ms마다 조작값 전송
    if (now - lastTx >= TX_INTERVAL) {
      lastTx = now;
      sendValue();
    }
  }

  // 통신 복구 처리
  checkRecovery();

  // OLED 갱신
  if (now - lastUi >= UI_INTERVAL) {
    lastUi = now;
    drawOLED();
  }

  // 2초마다 시리얼 진단값 출력
  if (now - lastDiag >= 2000) {
    lastDiag = now;

    Serial.print(F("ADC="));
    Serial.print(adcValue);

    Serial.print(F(" POT="));
    Serial.print(potPercent);

    Serial.print(F("% TEC="));
    Serial.print(can.errorCountTX());

    Serial.print(F(" REC="));
    Serial.println(can.errorCountRX());
  }
}
