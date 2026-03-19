// comm.h
#ifndef COMM_H
#define COMM_H

#include <stdbool.h>
#include <stdint.h>

void Comm_Init(void);       // 수신 인터럽트 초기화

// 라즈베리파이 (USART2)
void Comm_Rasp_RxCallback(void); // HAL_UART_RxCpltCallback에서 호출
bool Comm_IsStartFlagSet(void);
void Comm_ClearStartFlag(void);
void Comm_SendDone(void);   // 작업 완료 신호 PC/라즈베리파이로 전송

// 아두이노 (USART1)
void Comm_Arduino_RxCallback(void);
bool Comm_IsResetFlagSet(void);
void Comm_ClearResetFlag(void);
void Comm_SendScara(void);  // 아두이노로 전송

// 테스트할때 사용
void Comm_SetStartFlag(void);   // 외부(test.c)에서 직접 세팅용

#endif
