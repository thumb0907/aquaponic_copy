#ifndef MY_SERIAL_PROTOCOL_H
#define MY_SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <stdint.h>

// =========================
// 전역 플래그
// =========================
extern uint8_t ssf;
extern uint8_t smf;
extern uint8_t crf;
extern uint16_t uv;
extern uint8_t hm;

extern uint16_t wcnt;
extern uint8_t ulf;
extern uint8_t urf;
extern uint8_t wrf;
extern uint8_t wlf;
extern uint8_t uef;
extern uint8_t wef;
extern uint8_t ff;

// sect2 / sect3 명령과 이번 작업의 좌우 슬롯 선택값
extern uint8_t s2f;
extern uint8_t s3f;
extern uint8_t scara_src;
extern uint8_t scara_dst;

static const uint8_t SOF = 0xAA;
static const uint8_t MAX_DATA_LEN = 16;

// =========================
// ID 정의
// =========================
enum ParamId : uint8_t {
  PID_SSF  = 0x01,
  PID_SMF  = 0x02,
  PID_CRF  = 0x03,
  PID_UV   = 0x04,

  PID_ULF = 0x05,
  PID_URF  = 0x06,
  PID_WCNT  = 0x07,
  PID_WLF  = 0x08,
  PID_WRF  = 0x09,
  PID_FF  = 0x0A,
  PID_UEF  = 0x0B,
  PID_WEF   = 0x0C,
  PID_ESTOP = 0x10,
  PID_RESET = 0x11,
  PID_HM = 0x12,

  PID_S2F       = 0x14,
  PID_S3F       = 0x15,
  PID_SCARA_SRC = 0x16,
  PID_SCARA_DST = 0x17
};

// master_node.py의 SLOT_NONE / SLOT_LEFT / SLOT_RIGHT와 동일하다.
enum ScaraSlotCode : uint8_t {
  SCARA_SLOT_NONE  = 0x00,
  SCARA_SLOT_LEFT  = 0x01,
  SCARA_SLOT_RIGHT = 0x02
};

void setflag();

// =========================
// 초기화 / 주기 실행
// =========================
void serialProtocolBegin(unsigned long baud);
void serialReceiveTask(void);

// =========================
// 송신 함수
// =========================
void sendFrame(uint8_t id, const uint8_t* data, uint8_t len);
void sendU8(uint8_t id, uint8_t value);
void sendU16(uint8_t id, uint16_t value);

// =========================
// 현재 값 getter / setter
// =========================
uint8_t getSsf(void);
uint8_t getSmf(void);
uint8_t getCrf(void);
uint16_t getUv(void);
uint8_t getHm(void);
void setHm(uint8_t value);

uint8_t getWcnt(void);
uint8_t getUlf(void);
uint8_t getUrf(void);
uint8_t getWrf(void);
uint8_t getWlf(void);
uint8_t getUef(void);
uint8_t getWef(void);
uint8_t getFf(void);

uint8_t getS2f(void);
uint8_t getS3f(void);
uint8_t getScaraSrc(void);
uint8_t getScaraDst(void);

// 실제 모터 정지/원점 복귀는 동작 계층이 이 요청을 소비해 수행한다.
bool isEstopRequested(void);
bool isResetRequested(void);
void clearEstopRequest(void);
void clearResetRequest(void);

void setSsf(uint8_t value);
void setSmf(uint8_t value);
void setCrf(uint8_t value);
void setUv(uint16_t value);

void setWcnt(uint8_t value);
void setUlf(uint8_t value);
void setUrf(uint8_t value);
void setWrf(uint8_t value);
void setWlf(uint8_t value);
void setUef(uint8_t value);
void setWef(uint8_t value);
void setFf(uint8_t value);

void setS2f(uint8_t value);
void setS3f(uint8_t value);
void setScaraSrc(uint8_t value);
void setScaraDst(uint8_t value);

// =========================
// 이전 값 관련
// =========================
uint8_t getPrevSsf(void);
uint8_t getPrevCrf(void);
uint8_t getPrevHm(void);

uint8_t getPrevWcnt(void);
uint8_t getPrevUlf(void);
uint8_t getPrevUrf(void);
uint8_t getPrevWrf(void);
uint8_t getPrevWlf(void);
uint8_t getPrevUef(void);
uint8_t getPrevWef(void);
uint8_t getPrevFf(void);
uint8_t getPrevS2f(void);
uint8_t getPrevS3f(void);

void updatePrevFlags(void);

// =========================
// 편의 송신 함수
// =========================
void sendSsf(void);
void sendSmf(void);
void sendCrf(void);
void sendUv(void);
void sendHm(void);

void sendWcnt(void);
void sendUlf(void);
void sendUrf(void);
void sendWrf(void);
void sendWlf(void);
void sendUef(void);
void sendWef(void);
void sendFf(void);
void sendS2f(void);
void sendS3f(void);
void sendScaraSrc(void);
void sendScaraDst(void);

#endif
