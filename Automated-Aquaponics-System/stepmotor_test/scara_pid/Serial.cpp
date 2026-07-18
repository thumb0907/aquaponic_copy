#include "Serial.h"

// =========================
// 내부 상태 변수
// =========================
uint8_t ssf = 0;
uint8_t smf = 0;
uint8_t crf = 0;
uint8_t hm = 0;
uint16_t uv = 0;


uint16_t wcnt = 0;
uint8_t ulf  = 0;
uint8_t urf  = 0;
uint8_t wrf  = 0;
uint8_t wlf  = 0;
uint8_t uef  = 0;
uint8_t wef  = 0;
uint8_t ff   = 0;

uint8_t s2f = 0;
uint8_t s3f = 0;
uint8_t scara_src = SCARA_SLOT_NONE;
uint8_t scara_dst = SCARA_SLOT_NONE;

static uint8_t prev_hm = 0;
static uint8_t prev_ssf = 0;
static uint8_t prev_crf = 0;

static uint16_t prev_wcnt = 0;
static uint8_t prev_ulf  = 0;
static uint8_t prev_urf  = 0;
static uint8_t prev_wrf  = 0;
static uint8_t prev_wlf  = 0;
static uint8_t prev_uef  = 0;
static uint8_t prev_wef  = 0;
static uint8_t prev_ff   = 0;
static uint8_t prev_s2f  = 0;
static uint8_t prev_s3f  = 0;
static bool estop_requested = false;
static bool reset_requested = false;

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

  setS2f(0);
  setS3f(0);
  setScaraSrc(SCARA_SLOT_NONE);
  setScaraDst(SCARA_SLOT_NONE);
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
static void handleFrame(uint8_t id, const uint8_t* data, uint8_t len) {
  switch (id) {
    case PID_SSF:
      if (len == 1) ssf = data[0];
      break;

    case PID_SMF:
      if (len == 1) smf = data[0];
      break;

    case PID_CRF:
      if (len == 1) crf = data[0];
      break;

    case PID_HM:
      if (len == 1) hm = data[0];
      break;

    case PID_UV:
      if (len == 2) {
        uv = ((uint16_t)data[0] << 8) | data[1];
      }
      break;

    case PID_WCNT:
      if (len == 1) wcnt = data[0];
      break;

    case PID_ULF:
      if (len == 1) ulf = data[0];
      break;

    case PID_URF:
      if (len == 1) urf = data[0];
      break;

    case PID_WRF:
      if (len == 1) wrf = data[0];
      break;

    case PID_WLF:
      if (len == 1) wlf = data[0];
      break;

    case PID_UEF:
      if (len == 1) uef = data[0];
      break;

    case PID_WEF:
      if (len == 1) wef = data[0];
      break;

    case PID_FF:
      if (len == 1) ff = data[0];
      break;

    case PID_S2F:
      if (len == 1 && data[0] <= 1) s2f = data[0];
      break;

    case PID_S3F:
      if (len == 1 && data[0] <= 1) s3f = data[0];
      break;

    case PID_SCARA_SRC:
      if (len == 1 && data[0] <= SCARA_SLOT_RIGHT) {
        scara_src = data[0];
      }
      break;

    case PID_SCARA_DST:
      if (len == 1 && data[0] <= SCARA_SLOT_RIGHT) {
        scara_dst = data[0];
      }
      break;

    case PID_ESTOP:
      if (len == 0) estop_requested = true;
      break;

    case PID_RESET:
      if (len == 0) reset_requested = true;
      break;

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

void serialReceiveTask(void) {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();

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

// =========================
// getter
// =========================
uint8_t getSsf(void) { return ssf; }
uint8_t getSmf(void) { return smf; }
uint8_t getCrf(void) { return crf; }
uint16_t getUv(void) { return uv; }
uint8_t getHm(void) { return hm; }

uint8_t getWcnt(void) { return wcnt; }
uint8_t getUlf(void)  { return ulf; }
uint8_t getUrf(void)  { return urf; }
uint8_t getWrf(void)  { return wrf; }
uint8_t getWlf(void)  { return wlf; }
uint8_t getUef(void)  { return uef; }
uint8_t getWef(void)  { return wef; }
uint8_t getFf(void)   { return ff; }

uint8_t getS2f(void)       { return s2f; }
uint8_t getS3f(void)       { return s3f; }
uint8_t getScaraSrc(void)  { return scara_src; }
uint8_t getScaraDst(void)  { return scara_dst; }

bool isEstopRequested(void) { return estop_requested; }
bool isResetRequested(void) { return reset_requested; }
void clearEstopRequest(void) { estop_requested = false; }
void clearResetRequest(void) { reset_requested = false; }

// =========================
// setter
// =========================
void setSsf(uint8_t value) { ssf = value; }
void setSmf(uint8_t value) { smf = value; }
void setCrf(uint8_t value) { crf = value; }
void setUv(uint16_t value) { uv = value; }
void setHm(uint8_t value) { hm = value; }

void setWcnt(uint8_t value) { wcnt = value; }
void setUlf(uint8_t value)  { ulf = value; }
void setUrf(uint8_t value)  { urf = value; }
void setWrf(uint8_t value)  { wrf = value; }
void setWlf(uint8_t value)  { wlf = value; }
void setUef(uint8_t value)  { uef = value; }
void setWef(uint8_t value)  { wef = value; }
void setFf(uint8_t value)   { ff = value; }

void setS2f(uint8_t value)      { s2f = value; }
void setS3f(uint8_t value)      { s3f = value; }
void setScaraSrc(uint8_t value) { scara_src = value; }
void setScaraDst(uint8_t value) { scara_dst = value; }

// =========================
// 이전 값 관련
// =========================
uint8_t getPrevSsf(void) { return prev_ssf; }
uint8_t getPrevCrf(void) { return prev_crf; }
uint8_t getPrevHm(void) { return prev_hm; }

uint8_t getPrevWcnt(void) { return prev_wcnt; }
uint8_t getPrevUlf(void)  { return prev_ulf; }
uint8_t getPrevUrf(void)  { return prev_urf; }
uint8_t getPrevWrf(void)  { return prev_wrf; }
uint8_t getPrevWlf(void)  { return prev_wlf; }
uint8_t getPrevUef(void)  { return prev_uef; }
uint8_t getPrevWef(void)  { return prev_wef; }
uint8_t getPrevFf(void)   { return prev_ff; }
uint8_t getPrevS2f(void)  { return prev_s2f; }
uint8_t getPrevS3f(void)  { return prev_s3f; }

void updatePrevFlags(void) {
  prev_ssf = ssf;
  prev_crf = crf;
  prev_hm = hm;

  prev_wcnt = wcnt;
  prev_ulf  = ulf;
  prev_urf  = urf;
  prev_wrf  = wrf;
  prev_wlf  = wlf;
  prev_uef  = uef;
  prev_wef  = wef;
  prev_ff   = ff;
  prev_s2f  = s2f;
  prev_s3f  = s3f;
}

// =========================
// 편의 송신 함수
// =========================
void sendSsf(void)  { sendU8(PID_SSF, ssf); }
void sendSmf(void)  { sendU8(PID_SMF, smf); }
void sendCrf(void)  { sendU8(PID_CRF, crf); }
void sendUv(void)   { sendU16(PID_UV, uv); }
void sendHm(void) { sendU8(PID_HM, hm); }

void sendWcnt(void) { sendU8(PID_WCNT, wcnt); }
void sendUlf(void)  { sendU8(PID_ULF, ulf); }
void sendUrf(void)  { sendU8(PID_URF, urf); }
void sendWrf(void)  { sendU8(PID_WRF, wrf); }
void sendWlf(void)  { sendU8(PID_WLF, wlf); }
void sendUef(void)  { sendU8(PID_UEF, uef); }
void sendWef(void)  { sendU8(PID_WEF, wef); }
void sendFf(void)   { sendU8(PID_FF, ff); }
void sendS2f(void)  { sendU8(PID_S2F, s2f); }
void sendS3f(void)  { sendU8(PID_S3F, s3f); }
void sendScaraSrc(void) { sendU8(PID_SCARA_SRC, scara_src); }
void sendScaraDst(void) { sendU8(PID_SCARA_DST, scara_dst); }
