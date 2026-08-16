 #ifndef TEST_H
#define TEST_H

/*
 * TEST_MODE  1 = 테스트 모드 (USART2로 PC 터미널 명령 수신)
 *            0 = 정상 시퀀스 모드
 *
 */
#define TEST_MODE   0

#define SKIP_COMM   0 // 1이면 매니퓰레이터 통신 x

void Test_Init(void);
void Test_Run(void);
void Test_RxCallback(void);   /* main.c HAL_UART_RxCpltCallback 에서 호출 */

#endif /* TEST_H */
