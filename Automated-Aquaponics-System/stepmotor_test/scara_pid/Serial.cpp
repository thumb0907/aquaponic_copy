#include "Serial.h"

// =========================
// 내부 상태 변수
// =========================
static uint8_t hmf = 0;
static uint8_t ssf = 0;
static uint8_t s2f = 0;
static uint8_t s3f = 0;
static uint8_t smf = 0;

static uint16_t uv = 0;
static uint8_t scaraSrc = SCARA_SLOT_NONE;
static uint8_t scaraDst = SCARA_SLOT_NONE;

static uint8_t prevHmf = 0;
static uint8_t prevSsf = 0;
static uint8_t prevS2f = 0;
static uint8_t prevS3f = 0;

static bool estopRequested = false;
static bool resetRequested = false;

void setflag()
{
  setSsf(0);
  setSmf(0);
  setCrf(0);
  setUv(0);
  setHm(0);

  setWcnt(0);
  setUlf(0);
  setUrf(0);
  setWrf(0);
  setWlf(0);
  setUef(0);
  setWef(0);
  setFf(0);
}

// =========================
// 수신 상태 머신 변수
// =========================
enum RxState : uint8_t {
  RX_WAIT_SOF,
  RX_WAIT_ID,
  RX_WAIT_LEN,
  RX_WAIT_DATA,
  RX_WAIT_CHK
};

static RxState rxState = RX_WAIT_SOF;
static uint8_t rxId = 0;
static uint8_t rxLen = 0;
static uint8_t rxData[MAX_DATA_LEN];
static uint8_t rxIndex = 0;

// =========================
// 체크섬 계산
// =========================
static uint8_t calcChecksum(uint8_t id, uint8_t len, const uint8_t* data) {
  uint16_t sum = id + len;

  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
  }

  return (uint8_t)(sum & 0xFF);
}

// =========================
// 내부 프레임 반영
// =========================
static void handleFrame(
  uint8_t pid,
  const uint8_t* data,
  uint8_t len
) {
  if (pid == PID_ESTOP && len == 0) {
    estopRequested = true;
    return;
  }

  if (pid == PID_RESET && len == 0) {
    resetRequested = true;
    return;
  }

  if (pid == PID_UV && len == 2) {
    uv = ((uint16_t)data[0] << 8) | data[1];
    return;
  }

  if (len != 1) {
    return;
  }

  const uint8_t value = data[0];

  switch (pid) {
    case PID_HMF:
      if (value <= 1) hmf = value;
      break;

    case PID_SSF:
      if (value <= 1) ssf = value;
      break;

    case PID_S2F:
      if (value <= 1) s2f = value;
      break;

    case PID_S3F:
      if (value <= 1) s3f = value;
      break;

    case PID_SCARA_SRC:
      if (value <= SCARA_SLOT_RIGHT) {
        scaraSrc = value;
      }
      break;

    case PID_SCARA_DST:
      if (value <= SCARA_SLOT_RIGHT) {
        scaraDst = value;
      }
      break;

    // ULF, URF, WLF, WRF 등 기존 처리 유지

    default:
      break;
  }
}

// =========================
// 외부 공개 함수
// =========================
void serialProtocolBegin(unsigned long baud) {
  Serial.begin(baud);
}

void sendFrame(uint8_t id, const uint8_t* data, uint8_t len) {
  uint8_t chk = calcChecksum(id, len, data);

  Serial.write(SOF);
  Serial.write(id);
  Serial.write(len);

  for (uint8_t i = 0; i < len; i++) {
    Serial.write(data[i]);
  }

  Serial.write(chk);
}

void sendU8(uint8_t id, uint8_t value) {
  uint8_t buf[1] = { value };
  sendFrame(id, buf, 1);
}

void sendU16(uint8_t id, uint16_t value) {
  uint8_t buf[2];
  buf[0] = (uint8_t)((value >> 8) & 0xFF);
  buf[1] = (uint8_t)(value & 0xFF);
  sendFrame(id, buf, 2);
}

static unsigned long lastRxByteAt = 0;
static const unsigned long RX_TIMEOUT_MS = 100;

static void resetRxParser() {
  rxState = RX_WAIT_SOF;
  rxIndex = 0;
  rxLen = 0;
}
void serialReceiveTask(void) {
  if (
    rxState != RX_WAIT_SOF &&
    millis() - lastRxByteAt > RX_TIMEOUT_MS
  ) {
    resetRxParser();
  }

  while (Serial.available() > 0) {
    const uint8_t b = (uint8_t)Serial.read();
    lastRxByteAt = millis();

    switch (rxState) {
      case RX_WAIT_SOF:
        if (b == SOF) {
          rxState = RX_WAIT_ID;
        }
        break;

      case RX_WAIT_ID:
        rxId = b;
        rxState = RX_WAIT_LEN;
        break;

      case RX_WAIT_LEN:
        rxLen = b;

        if (rxLen > MAX_DATA_LEN) {
          rxState = RX_WAIT_SOF;
          rxIndex = 0;
        } else if (rxLen == 0) {
          rxState = RX_WAIT_CHK;
        } else {
          rxIndex = 0;
          rxState = RX_WAIT_DATA;
        }
        break;

      case RX_WAIT_DATA:
        rxData[rxIndex++] = b;

        if (rxIndex >= rxLen) {
          rxState = RX_WAIT_CHK;
        }
        break;

      case RX_WAIT_CHK: {
        uint8_t recvChk = b;
        uint8_t calcChk = calcChecksum(rxId, rxLen, rxData);

        if (recvChk == calcChk) {
          handleFrame(rxId, rxData, rxLen);
        }

        rxState = RX_WAIT_SOF;
        rxIndex = 0;
        break;
      }
    }
  }
}
uint8_t getHmf() { return hmf; }
uint8_t getSsf() { return ssf; }
uint8_t getS2f() { return s2f; }
uint8_t getS3f() { return s3f; }
uint8_t getSmf() { return smf; }
uint16_t getUv() { return uv; }

uint8_t getScaraSrc() { return scaraSrc; }
uint8_t getScaraDst() { return scaraDst; }

uint8_t getPrevHmf() { return prevHmf; }
uint8_t getPrevSsf() { return prevSsf; }
uint8_t getPrevS2f() { return prevS2f; }
uint8_t getPrevS3f() { return prevS3f; }

bool isEstopRequested() { return estopRequested; }
bool isResetRequested() { return resetRequested; }

void clearEstopRequest() { estopRequested = false; }
void clearResetRequest() { resetRequested = false; }

void setHmf(uint8_t value) { hmf = value; }
void setSsf(uint8_t value) { ssf = value; }
void setS2f(uint8_t value) { s2f = value; }
void setS3f(uint8_t value) { s3f = value; }
void setSmf(uint8_t value) { smf = value; }

void updatePrevFlags() {
  prevHmf = hmf;
  prevSsf = ssf;
  prevS2f = s2f;
  prevS3f = s3f;
}

void sendState(uint8_t state) {
  sendU8(PID_STATE, state);
}

void sendHmf() { sendU8(PID_HMF, hmf); }
void sendSsf() { sendU8(PID_SSF, ssf); }
void sendS2f() { sendU8(PID_S2F, s2f); }
void sendS3f() { sendU8(PID_S3F, s3f); }
void sendSmf() { sendU8(PID_SMF, smf); }

void setflag() {
  hmf = 0;
  ssf = 0;
  s2f = 0;
  s3f = 0;
  smf = 0;

  uv = 0;
  scaraSrc = SCARA_SLOT_NONE;
  scaraDst = SCARA_SLOT_NONE;

  prevHmf = 0;
  prevSsf = 0;
  prevS2f = 0;
  prevS3f = 0;

  estopRequested = false;
  resetRequested = false;
}