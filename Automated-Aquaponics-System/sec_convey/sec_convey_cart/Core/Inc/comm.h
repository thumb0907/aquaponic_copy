// comm.h — STM2 통신 레이어 (바이너리 프레임 프로토콜)
//
// [프레임 구조]  SOF(1) | ID(1) | LEN(1) | DATA(LEN) | CHK(1)
//   SOF = 0xAA
//   CHK = (ID + LEN + sum(DATA)) & 0xFF
//
// [수신] 라파2 → STM2
//   PID_FF    : 1=사이클 시작, 0=초기화
//   PID_HF    : 1=수확 완료 (라파2가 스카라+매니퓰 R1 취합 후 전송)
//   PID_ESTOP : 긴급정지
//   PID_RESET : 전체 리셋
//
// [송신] STM2 → 라파2
//   PID_STATE : 상태 보고 (STATE_* 값)
//   PID_DONE  : 사이클 완료 (DONE_CYCLE2)
//   PID_ERR   : 에러 보고 (ERR_* 값)

#ifndef COMM_H
#define COMM_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ── 프로토콜 상수 ──────────────────── */
#define COMM_SOF          0xAA
#define COMM_MAX_DATA_LEN 16

/* ── 수신 플래그 ID ─────────────────── */
#define PID_FF    0x0A   // fix flag     (1=시작, 0=초기화)
#define PID_HF    0x0D   // harvest flag (1=수확완료)
#define PID_ESTOP 0x10   // 긴급정지
#define PID_RESET 0x11   // 리셋

/* ── 송신 보고 ID ───────────────────── */
#define PID_STATE 0x20   // 상태 보고
#define PID_DONE  0x21   // 완료 보고
#define PID_ERR   0x22   // 에러 보고

/* ── STATE 값 (STM2 → 라파2) ────────── */
#define STATE_IDLE         0x01
#define STATE_CONVEY_RUN   0x02
#define STATE_IR_DETECTED  0x03
#define STATE_Z_FIX        0x04
#define STATE_HARVESTING   0x05
#define STATE_EJECTING     0x06
#define STATE_EJECT_DONE   0x07
#define STATE_ESTOP        0x08

/* ── DONE 값 ────────────────────────── */
#define DONE_CYCLE2  0x01   // pi2_node 'CYCLE2' 문자열과 일치해야 함

/* ── ERR 코드 ───────────────────────── */
#define ERR_CONVEY_TIMEOUT   0x01   // IR 미감지 타임아웃
#define ERR_HOMING_TIMEOUT   0x02   // 리밋스위치 미감지 타임아웃
#define ERR_Z_FIX_TIMEOUT    0x03   // Z 고정 이동 타임아웃
#define ERR_HARVEST_TIMEOUT  0x04   // 수확 완료 신호 타임아웃
#define ERR_Z_RETURN_TIMEOUT 0x05   // Z 복귀 타임아웃
#define ERR_EXIT_TIMEOUT     0x06   // 배출 타임아웃

/* ── 초기화 / 콜백 ──────────────────── */
void Comm_Init(void);
void Comm_Rasp_RxCallback(void);   // main.c HAL_UART_RxCpltCallback에서 호출

/* ── 플래그 getter / setter ─────────── */
uint8_t Comm_GetFf(void);          // FF 값 (0 또는 1)
void    Comm_SetFf(uint8_t v);

bool    Comm_IsHfSet(void);        // 수확 완료 플래그
void    Comm_ClearHfFlag(void);

bool    Comm_IsEstop(void);        // 긴급정지 플래그
void    Comm_ClearEstop(void);

bool    Comm_IsReset(void);        // 리셋 플래그
void    Comm_ClearReset(void);

void    Comm_ClearAllFlags(void);  // 전체 초기화

/* ── 송신 ───────────────────────────── */
void Comm_SendState(uint8_t state);         // 상태 보고
void Comm_SendDone(uint8_t done_code);      // 완료 보고 (DONE_CYCLE2)
void Comm_SendError(uint8_t err_code);      // 에러 보고

#endif
