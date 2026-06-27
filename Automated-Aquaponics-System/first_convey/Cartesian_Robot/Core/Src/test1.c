// test.c (첫번째 컨베이어 프로젝트 - 전체동작 테스트 포함)
// ─────────────────────────────────────────────────────────────
// TEST_MODE=1  : 개별 동작 확인
// TEST_MODE=0, FULL_SEQ_TEST=1 : S1 보내면 전체 시퀀스 구동
//
// 커맨드 (HEX: 두 글자 + 0A)
//   개별 동작 (TEST_MODE=1 전용)
//   C1  (43 31 0A)  컨베이어 구동
//   C0  (43 30 0A)  컨베이어 정지
//   XF  (58 46 0A)  X축 + 이동 (1000스텝)
//   XB  (58 42 0A)  X축 - 이동 (1000스텝)
//   XS  (58 53 0A)  X축 정지
//   ZF  (5A 46 0A)  Z축 + 이동 (1000스텝)
//   ZB  (5A 42 0A)  Z축 - 이동 (1000스텝)
//   ZS  (5A 53 0A)  Z축 정지
//   HM  (48 4D 0A)  홈잉 시작
//   IR  (49 52 0A)  IR 센서 상태
//   XL  (58 4C 0A)  X 리밋 상태
//   ZL  (5A 4C 0A)  Z 리밋 상태
//   PU  (50 55 0A)  펌프 5초 ON 후 OFF
//   PK  (50 4B 0A)  픽업 테스트: X리밋→백오프→Z90하강→펌프ON→Z상승→펌프OFF
//   PW  (50 57 0A)  픽업+파종 테스트: 픽업→X240이동→Z40하강→펌프OFF→Z상승
//
//   공통 (TEST_MODE, FULL_SEQ_TEST 모두 사용 가능)
//   S1  (53 31 0A)  전체 시퀀스 시작
//   E1  (45 31 0A)  긴급정지
// ─────────────────────────────────────────────────────────────

#include "test1.h"
#include "sequence.h"
#include "main.h"
#include "first_convey.h"
#include "cartesian.h"
#include "sensor.h"
#include "board_pin.h"
#include "comm.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

#define TEST_BUF_SIZE   8
#define TEST_X_STEPS    2000
#define TEST_Z_STEPS    2000
#define TEST_X_HZ       1400
#define TEST_Z_HZ       2600

static uint8_t test_rx_byte  = 0;
static uint8_t test_buf[TEST_BUF_SIZE]     = {0};
static uint8_t test_buf_idx  = 0;
static volatile bool test_cmd_ready = false;
static uint8_t test_cmd_buf[TEST_BUF_SIZE] = {0};
static bool test_cartesian_running = false;

// ── 송신 ─────────────────────────────────────
static void Test_Send(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
}

// ── 커맨드 처리 ──────────────────────────────
static void Test_Process(uint8_t *buf)
{
    char tmp[48];

#if TEST_MODE
    // ── 개별 동작 커맨드 ─────────────────────
    if (buf[0] == 'C' && buf[1] == '1') {
        FirstConvey_Reset();
        Test_Send("[OK] Conveyor START");
        return;
    }
    else if (buf[0] == 'C' && buf[1] == '0') {
        FirstConvey_ForceStop();
        Test_Send("[OK] Conveyor STOP");
        return;
    }
    else if (buf[0] == 'X' && buf[1] == 'F') {
        Cartesian_TestMoveX(+TEST_X_STEPS, TEST_X_HZ);
        Test_Send("[OK] X + (forward)");
        return;
    }
    else if (buf[0] == 'X' && buf[1] == 'B') {
        Cartesian_TestMoveX(-TEST_X_STEPS, TEST_X_HZ);
        Test_Send("[OK] X - (back)");
        return;
    }
    else if (buf[0] == 'X' && buf[1] == 'S') {
        Cartesian_TestStopX();
        Test_Send("[OK] X STOP");
        return;
    }
    else if (buf[0] == 'Z' && buf[1] == 'F') {
        Cartesian_TestMoveZ(+TEST_Z_STEPS, TEST_Z_HZ);
        Test_Send("[OK] Z + (down)");
        return;
    }
    else if (buf[0] == 'Z' && buf[1] == 'B') {
        Cartesian_TestMoveZ(-TEST_Z_STEPS, TEST_Z_HZ);
        Test_Send("[OK] Z - (up)");
        return;
    }
    else if (buf[0] == 'Z' && buf[1] == 'S') {
        Cartesian_TestStopZ();
        Test_Send("[OK] Z STOP");
        return;
    }
    else if (buf[0] == 'H' && buf[1] == 'M') {
        Cartesian_ResetSequence();
        Cartesian_StartHoming();
        Test_Send("[OK] HOMING START");
        return;
    }
    else if (buf[0] == 'I' && buf[1] == 'R') {
        if (Sensor_IR_Detected())
            Test_Send("[IR] DETECTED");
        else
            Test_Send("[IR] not detected");
        return;
    }
    else if (buf[0] == 'X' && buf[1] == 'L') {
        if (HAL_GPIO_ReadPin(X_LIM_PORT, X_LIM_PIN) == GPIO_PIN_RESET)
            Test_Send("[XLIM] HIT");
        else
            Test_Send("[XLIM] not hit");
        return;
    }
    else if (buf[0] == 'Z' && buf[1] == 'L') {
        if (HAL_GPIO_ReadPin(Z_LIM_PORT, Z_LIM_PIN) == GPIO_PIN_RESET)
            Test_Send("[ZLIM] HIT");
        else
            Test_Send("[ZLIM] not hit");
        return;
    }
    else if (buf[0] == 'P' && buf[1] == 'U') {
        // 펌프 5초 ON 후 OFF (블로킹)
        Cartesian_TestPump();
        Test_Send("[OK] PUMP 5s ON->OFF");
        return;
    }
    else if (buf[0] == 'P' && buf[1] == 'K') {
        // 픽업 한 사이클 시작 (논블로킹, Sequence_Task/Cartesian_Task가 진행)
        Cartesian_TestPickCycle();
        test_cartesian_running = true;
        Test_Send("[OK] PICK CYCLE START");
        return;
    }
    else if (buf[0] == 'P' && buf[1] == 'W') {
        // 픽업+파종 한 사이클 시작 (논블로킹)
        Cartesian_TestPickSowCycle();
        test_cartesian_running = true;
        Test_Send("[OK] PICK+SOW CYCLE START");
        return;
    }
    else
#endif

    // ── 공통 커맨드 (TEST_MODE, FULL_SEQ_TEST 모두) ──
    if (buf[0] == 'S' && buf[1] == '1') {
    	Comm_SetC1f(1);
        Test_Send("[OK] SEQUENCE START");
        return;
    }
    else if (buf[0] == 'E' && buf[1] == '1') {
    	test_cartesian_running = false;
        FirstConvey_ForceStop();
        Cartesian_TestStopX();
        Cartesian_TestStopZ();
        Test_Send("[ESTOP] ALL STOP");
        return;
    }
    else {
        Test_Send("[ERR] unknown cmd");
    }
}

// ── UART 수신 콜백 ───────────────────────────
void Test_RxCallback(void)
{
    if (test_rx_byte == '\n') {
        test_buf[test_buf_idx] = '\0';

        if (test_buf_idx > 0) {
            // 앞쪽 공백 제거
            uint8_t start = 0;
            while (test_buf[start] == ' ' || test_buf[start] == '\t') {
                start++;
            }

            // 뒤쪽 공백 제거
            int end = strlen((char *)test_buf) - 1;
            while (end >= 0 &&
                   (test_buf[end] == ' ' || test_buf[end] == '\t')) {
                test_buf[end] = '\0';
                end--;
            }

            memset(test_cmd_buf, 0, TEST_BUF_SIZE);
            strncpy((char *)test_cmd_buf, (char *)&test_buf[start], TEST_BUF_SIZE - 1);

            if (test_cmd_buf[0] != '\0') {
                test_cmd_ready = true;
            }
        }

        memset(test_buf, 0, TEST_BUF_SIZE);
        test_buf_idx = 0;
    }
    else if (test_rx_byte == '\r') {
        // 무시
    }
    else if (test_buf_idx < TEST_BUF_SIZE - 1) {
        test_buf[test_buf_idx++] = test_rx_byte;
    }
    else {
        memset(test_buf, 0, TEST_BUF_SIZE);
        test_buf_idx = 0;
    }

    HAL_UART_Receive_IT(&huart2, &test_rx_byte, 1);
}
// ── 초기화 ───────────────────────────────────
void Test_Init(void)
{
#if TEST_MODE
    HAL_UART_Receive_IT(&huart2, &test_rx_byte, 1);
    Test_Send("=== TEST MODE ===");
    Test_Send("C1  Conveyor START");
    Test_Send("C0  Conveyor STOP");
    Test_Send("XF  X + 1000step");
    Test_Send("XB  X - 1000step");
    Test_Send("XS  X STOP");
    Test_Send("ZF  Z + 1000step");
    Test_Send("ZB  Z - 1000step");
    Test_Send("ZS  Z STOP");
    Test_Send("HM  HOMING");
    Test_Send("IR  IR sensor");
    Test_Send("XL  X limit switch");
    Test_Send("ZL  Z limit switch");
    Test_Send("PU  PUMP 5s ON->OFF");
    Test_Send("PK  PICK cycle");
    Test_Send("PW  PICK+SOW cycle");
    Test_Send("S1  SEQUENCE START");
    Test_Send("E1  ESTOP");
    Test_Send("====================");
#endif
    // TEST_MODE=0: 아무것도 안 함
    // Comm_Init()이 huart2 수신 인터럽트를 담당
}

// ── 메인 루프에서 호출 ───────────────────────
void Test_Run(void)
{
#if TEST_MODE
    if (Cartesian_IsHomingStarted() && !Cartesian_IsHomingDone()) {
        Cartesian_HomingTask();
    }

    if (test_cartesian_running) {
        Cartesian_Task();

        if (Cartesian_IsCycleDone()) {
            test_cartesian_running = false;
            Cartesian_ResetSequence();
            Test_Send("[DONE] TEST CARTESIAN CYCLE");
        }
    }
    else {
        Sequence_Task();
    }
#endif

    if (!test_cmd_ready) return;
    test_cmd_ready = false;
    Test_Process(test_cmd_buf);
}
