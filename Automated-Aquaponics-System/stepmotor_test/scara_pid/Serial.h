#ifndef MY_SERIAL_PROTOCOL_H
#define MY_SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <stdint.h>

static const uint8_t SOF = 0xAA;
static const uint8_t MAX_DATA_LEN = 16;

enum ParamId : uint8_t {
  PID_SSF  = 0x01,
  PID_SMF  = 0x02,
  PID_CRF  = 0x03,
  PID_UV   = 0x04,

  PID_ULF  = 0x05,
  PID_URF  = 0x06,
  PID_WCNT = 0x07,
  PID_WLF  = 0x08,
  PID_WRF  = 0x09,
  PID_FF   = 0x0A,
  PID_UEF  = 0x0B,
  PID_WEF  = 0x0C,

  PID_ESTOP = 0x10,
  PID_RESET = 0x11,
  PID_HMF   = 0x12,

  PID_S2F       = 0x14,
  PID_S3F       = 0x15,
  PID_SCARA_SRC = 0x16,
  PID_SCARA_DST = 0x17,

  PID_STATE = 0x20,
  PID_DONE  = 0x21,
  PID_ERR   = 0x22
};

enum ScaraStateCode : uint8_t {
  SCARA_STATE_IDLE   = 0x01,
  SCARA_STATE_MOVING = 0x02,
  SCARA_STATE_DONE   = 0x03,
  SCARA_STATE_ESTOP  = 0x04
};

enum ScaraSlotCode : uint8_t {
  SCARA_SLOT_NONE  = 0x00,
  SCARA_SLOT_LEFT  = 0x01,
  SCARA_SLOT_RIGHT = 0x02
};

void setflag();

void serialProtocolBegin(unsigned long baud);
void serialReceiveTask();

void sendFrame(uint8_t pid, const uint8_t* data, uint8_t len);
void sendU8(uint8_t pid, uint8_t value);
void sendU16(uint8_t pid, uint16_t value);
void sendState(uint8_t state);

uint8_t getSsf();
uint8_t getSmf();
//uint8_t getCrf();
uint16_t getUv();
uint8_t getHmf();

uint8_t getS2f();
uint8_t getS3f();
uint8_t getScaraSrc();
uint8_t getScaraDst();

bool isEstopRequested();
bool isResetRequested();
void clearEstopRequest();
void clearResetRequest();

uint8_t getPrevHmf();
uint8_t getPrevSsf();
uint8_t getPrevS2f();
uint8_t getPrevS3f();

void updatePrevFlags();

void setSsf(uint8_t value);
void setSmf(uint8_t value);
void setHmf(uint8_t value);
void setS2f(uint8_t value);
void setS3f(uint8_t value);

void sendSsf();
void sendSmf();
void sendHmf();
void sendS2f();
void sendS3f();

#endif