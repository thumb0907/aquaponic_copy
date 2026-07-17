#ifndef TEST1_H
#define TEST1_H

// 모드 선택
// TEST_MODE  1 : 허큘러스 커맨드로 개별 동작 확인
// TEST_MODE  0 : 정상 시퀀스 모드(정상통신)
#define TEST_MODE     0

// FULL_SEQ_TEST 1 : 허큘러스에서 개별 시퀸스 구동
//               0 : 정상 시퀀스 (라즈베리파이/아두이노 통신 사용)
// TEST_MODE=0 일 때만 유효
#define FULL_SEQ_TEST 0


void Test_Init(void);
void Test_Run(void);
void Test_RxCallback(void);  // main.c HAL_UART_RxCpltCallback에서 호출

#endif
