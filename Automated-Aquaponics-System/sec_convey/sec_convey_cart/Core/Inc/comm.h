#ifndef COMM_H
#define COMM_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

/*
 * 수신 커맨드 (형식: "X1\n")
 *   USART1 스카라  : S1 = 트레이 올려놓음(시작)
 *                    R1 = 수확 완료
 *                    E1 = 긴급정지
 *   USART2 라파    : E1 = 긴급정지
 *   USART6 매니퓰  : R1 = 수확 완료
 *                    E1 = 긴급정지
 *
 * 송신
 *   STM32 → 스카라+매니퓰: H1\n (수확 시작)
 *   STM32 → 라파          : C1\n / I1\n / Z1\n / H1\n / R1\n / D1\n / E1\n
 */

void Comm_Init(void);

// 스카라 (USART1)
void Comm_Scara_RxCallback(void);
bool Comm_IsStartFlagSet(void);
void Comm_ClearStartFlag(void);
bool Comm_IsScaraDoneFlagSet(void);
void Comm_ClearScaraDoneFlag(void);

// 라즈베리파이 (USART2)
void Comm_Rasp_RxCallback(void);

// 매니퓰레이터 (USART6)
void Comm_Manip_RxCallback(void);
bool Comm_IsManipDoneFlagSet(void);
void Comm_ClearManipDoneFlag(void);

// 긴급정지 (어디서든 E1 오면 세팅)
bool Comm_IsEstopFlagSet(void);
void Comm_ClearEstopFlag(void);

// 송신
void Comm_SendHarvest(void);        // 스카라 + 매니퓰레이터: H1\n
void Comm_SendDone(void);           // 라파: D1\n
void Comm_SendErr(void);            // 전체: E1\n
void Comm_LogToRasp(const char *msg); // 라파 모니터링 로그

#endif
