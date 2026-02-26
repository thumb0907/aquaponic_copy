#ifndef COMM_H
#define COMM_H

#include <stdbool.h>
#include <stdint.h>

void Comm_Init(void);       // 수신 인터럽트 시작
void Comm_RxCallback(void); // HAL_UART_RxCpltCallback에서 호출
bool Comm_IsStartFlagSet(void);
void Comm_ClearStartFlag(void);
void Comm_SendDone(void);   // 작업 완료 신호 PC/라즈베리파이로 전송

#endif
