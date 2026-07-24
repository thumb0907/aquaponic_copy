#pragma once

#include <Arduino.h>

// PC Master와 약속된 PID
static constexpr uint8_t PID_ROBOT_JOB = 0x13;
static constexpr uint8_t PID_ESTOP     = 0x10;
static constexpr uint8_t PID_RESET     = 0x11;
static constexpr uint8_t PID_STATE     = 0x20;
static constexpr uint8_t PID_DONE      = 0x21;
static constexpr uint8_t PID_ERR       = 0x22;

// 작업 종류
static constexpr uint8_t JOB_HARVEST = 0x05;

// 매니퓰레이터 상태
static constexpr uint8_t MANIP_STATE_IDLE       = 0x01;
static constexpr uint8_t MANIP_STATE_HARVESTING = 0x02;
static constexpr uint8_t MANIP_STATE_ESTOP      = 0x03;
static constexpr uint8_t MANIP_STATE_ERROR      = 0x04;

// 오류 코드
static constexpr uint8_t MANIP_ERR_INVALID_JOB    = 0x01;
static constexpr uint8_t MANIP_ERR_BUSY           = 0x02;
static constexpr uint8_t MANIP_ERR_SEQUENCE_FAILED = 0x03;
static constexpr uint8_t MANIP_ERR_NOT_READY      = 0x04;
static constexpr uint8_t MANIP_ERR_ESTOP          = 0x05;

void commBegin(Stream &port);
void commPoll();

bool commTakeHarvestJob(uint8_t &job_id);
bool commTakeEstop();
bool commTakeReset();

void commSendState(uint8_t state, uint8_t job_id);
void commSendDone(uint8_t job_id);
void commSendError(uint8_t job_id, uint8_t error_code);