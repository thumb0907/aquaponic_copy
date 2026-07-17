/*
 * comm.h  ─  STM32 #1 통신 레이어
 *
 * [프레임 구조]
 *   SOF(1) | ID(1) | LEN(1) | DATA(LEN) | CHK(1)
 *   SOF = 0xAA
 *   CHK = (ID + LEN + sum(DATA)) & 0xFF
 *   SOF  = 0xAA         시작 신호. 프레임 시작을 알림
 *	 ID   = 플래그 종류   무슨 데이터인지 (SSF=0x01, CRF=0x03 등)
 *	 LEN  = 데이터 길이   DATA가 몇 바이트인지
 *	 DATA = 실제 값       SSF면 0 또는 1
 *	 CHK  = 체크섬        오류 검출용
 */

#ifndef COMM_H
#define COMM_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* ── 프로토콜 상수 ──────────────────────────── */
#define COMM_SOF          0xAA
#define COMM_MAX_DATA_LEN 16

/* ── 플래그 ID (양방향 공통) ─────────────────── */
#define PID_SSF   0x01   // scara start flag
#define PID_SMF   0x02   // scara move flag
#define PID_CRF   0x03   // cartesian reset flag
#define PID_UV    0x04   // uv실 트레이 개수
#define PID_ULF   0x05   // uv 왼쪽 발아 완료
#define PID_URF   0x06   // uv 오른쪽 발아 완료
#define PID_WCNT  0x07   // 수경재배실 트레이 개수
#define PID_WLF   0x08   // 수경재배실 왼쪽 성장
#define PID_WRF   0x09   // 수경재배실 오른쪽 성장
#define PID_FF    0x0A   // fix flag
#define PID_UEF   0x0B   // uv event flag
#define PID_WEF   0x0C   // water event flag
#define PID_HF    0x0D   // harvest flag
#define PID_C1F   0x0E   // conveyor1 flag
#define PID_C2F   0x0F   // conveyor2 flag
#define PID_ESTOP 0x10   // 긴급정지
#define PID_RESET 0x11   // 전체 리셋
#define PID_HMF   0x12   // scara homing flag

/* ── STM1 → 라파1 상태 보고 ID ──────────────── */
#define PID_STATE 0x20   // 상태 보고  1바이트
#define PID_DONE  0x21   // 완료 보고  1바이트
#define PID_ERR   0x22   // 에러 보고  1바이트

/* ── STATE 값 ───────────────────────────────── */
#define STATE_IDLE            0x01
#define STATE_HOMING          0x02
#define STATE_RUN_CONVEYOR1   0x03
#define STATE_SEEDING         0x04
#define STATE_EJECTING        0x05
#define STATE_WAIT_SCARA_PICK 0x06
#define STATE_ESTOP           0x07
#define STATE_PICKING		  0x08

/* ── DONE 값 ────────────────────────────────── */
#define DONE_HOMING  0x01
#define DONE_CYCLE1  0x02

/* ── ERR 값 ─────────────────────────────────── */
#define ERR_HOMING_TIMEOUT 0x01

/* ── 초기화 / 콜백 ──────────────────────────── */
void Comm_Init(void);
void Comm_Rasp_RxCallback(void);

/* ── 플래그 getter/setter ───────────────────── */
uint8_t  Comm_GetSsf(void);   void Comm_SetSsf(uint8_t v);
uint8_t  Comm_GetSmf(void);   void Comm_SetSmf(uint8_t v);
uint8_t  Comm_GetCrf(void);   void Comm_SetCrf(uint8_t v);
uint16_t Comm_GetUv(void);    void Comm_SetUv(uint16_t v);
uint8_t  Comm_GetUlf(void);   void Comm_SetUlf(uint8_t v);
uint8_t  Comm_GetUrf(void);   void Comm_SetUrf(uint8_t v);
uint16_t  Comm_GetWcnt(void);  void Comm_SetWcnt(uint16_t v);
uint8_t  Comm_GetWlf(void);   void Comm_SetWlf(uint8_t v);
uint8_t  Comm_GetWrf(void);   void Comm_SetWrf(uint8_t v);
uint8_t  Comm_GetFf(void);    void Comm_SetFf(uint8_t v);
uint8_t  Comm_GetUef(void);   void Comm_SetUef(uint8_t v);
uint8_t  Comm_GetWef(void);   void Comm_SetWef(uint8_t v);
uint8_t  Comm_GetHf(void);    void Comm_SetHf(uint8_t v);
uint8_t  Comm_GetC1f(void);   void Comm_SetC1f(uint8_t v);
uint8_t  Comm_GetC2f(void);   void Comm_SetC2f(uint8_t v);

bool Comm_IsEstop(void);      void Comm_ClearEstop(void);
bool Comm_IsReset(void);      void Comm_ClearReset(void);
bool Comm_IsCam1Detected(void); void Comm_ClearCam1Detected(void);
void Comm_ClearAllFlags(void);

/* ── 송신 ───────────────────────────────────── */
void Comm_SendState(uint8_t state_val);
void Comm_SendDone(uint8_t done_val);
void Comm_SendError(uint8_t err_val);
void Comm_SendFlag(uint8_t pid, uint8_t val);    // 1바이트 플래그 송신
void Comm_SendFlag16(uint8_t pid, uint16_t val); // 2바이트 플래그 송신

/* ── 테스트 전용 ─────────────────────────────── */
void Comm_SetStartFlag(void);

#endif /* COMM_H */
