// test.c
// ─────────────────────────────────────────────────────────────
// 허큘러스(PC) USART2 115200bps, 줄끝 LF(\n) 설정
//
// 커맨드 목록:
//   C1   컨베이어 수동 구동 (3500Hz)                  43 31 0A
//   C0   컨베이어 정지 및 사이클 취소              	  43 30 0A
//   CY   전체 한 사이클 실행                          43 59 0A
//       컨베이어 투입 → IR 감지 → Z축 하강·고정
//        → 수확 시뮬레이션 → Z축 복귀 → 컨베이어 배출
//
//   ZF   Z축 하강 (포토센서(리밋스위치)찍는 방향 방향, +3000스텝/내려감)  	 5A 46 0A
//   ZB   Z축 상승 (복귀 방향, -3000스텝/올라감)    5A 42 0A
//   ZS   Z축 즉시 정지                             5A 53 0A
//
//   IR   IR 센서 상태 출력                          49 52 0A
//   LM   Z축 포토센서 상태 출력                       4C 4D 0A
//   E1   전체 긴급정지 및 사이클 취소              	  45 31 0A
// ─────────────────────────────────────────────────────────────

#include "test.h"
#include "main.h"
#include "sec_convey.h"
#include "stepper.h"
#include "sensor.h"
#include <string.h>
#include "comm.h"

extern UART_HandleTypeDef huart2;

#define TEST_BUF_SIZE   8
#define TEST_CONV_HZ    3500
#define TEST_Z_STEPS    3000
#define TEST_Z_HZ       1000

static uint8_t test_rx_byte = 0;
static uint8_t test_buf[TEST_BUF_SIZE] = {0};
static uint8_t test_buf_idx = 0;
static volatile bool test_cmd_ready = false;
static uint8_t test_cmd_buf[TEST_BUF_SIZE] = {0};

/* ── CY 전체 사이클 설정 ───────────────────────── */
#define TEST_SETTLE_MS             200U
#define TEST_HARVEST_SIM_MS       3000U
#define TEST_EXIT_MIN_RUN_MS       500U

#define TEST_IR_TIMEOUT_MS       30000U
#define TEST_Z_TIMEOUT_MS        15000U
#define TEST_Z_RETURN_TIMEOUT_MS 12000U
#define TEST_EXIT_TIMEOUT_MS     15000U

/*
 * 현재 수동 명령 기준:
 *   +step = 내려감
 *   -step = 올라감
 *
 * 실제 방향이 반대라면 아래 두 부호를 서로 바꿔야 한다.
 */
#define TEST_Z_DESCEND_STEPS      15000
#define TEST_Z_RETURN_STEPS       5000
#define TEST_CYCLE_Z_HZ           1000U

typedef enum {
    TEST_CYCLE_IDLE = 0,
    TEST_CYCLE_WAIT_IR,
    TEST_CYCLE_SETTLE,
    TEST_CYCLE_Z_DESCEND,
    TEST_CYCLE_Z_HOLD,
    TEST_CYCLE_Z_RETURN,
    TEST_CYCLE_WAIT_EXIT
} TestCycleState;

static TestCycleState test_cycle_state = TEST_CYCLE_IDLE;
static uint32_t test_cycle_timer = 0;


/* ── 터미널 메시지 전송 ───────────────────────── */
static void Test_Send(const char *msg)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)msg,
        strlen(msg),
        100
    );

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)"\r\n",
        2,
        100
    );
}


/* ── CY 오류 처리 ─────────────────────────────── */
static void Test_CycleFail(const char *msg)
{
    Conveyor_Stop();
    Conveyor_Enable(false);
    Z_Stop();

    test_cycle_state = TEST_CYCLE_IDLE;

    Test_Send(msg);
}


/* ── CY 시작 ─────────────────────────────────── */
static void Test_ConveyorCycleStart(void)
{
    if (test_cycle_state != TEST_CYCLE_IDLE) {
        Test_Send("[ERR] Conveyor cycle busy");
        return;
    }

    Conveyor_Enable(true);
    Conveyor_SetDir(true);
    Conveyor_StartHz(TEST_CONV_HZ);

    test_cycle_timer = HAL_GetTick();
    test_cycle_state = TEST_CYCLE_WAIT_IR;

    Test_Send("[CYCLE] Conveyor input start");
}


/* ── CY 상태 머신 ─────────────────────────────── */
static void Test_ConveyorCycleRun(void)
{
    uint32_t now = HAL_GetTick();

    switch (test_cycle_state)
    {
        /* 아무 동작 없음 */
        case TEST_CYCLE_IDLE:
            break;


        /* 컨베이어 구동 → IR 감지 대기 */
        case TEST_CYCLE_WAIT_IR:
            if (IR_Detected()) {
                /* IR 감지 즉시 컨베이어 정지 */
                Conveyor_Stop();

                /* 드라이버는 활성 상태 유지 */
                Conveyor_Enable(true);

                test_cycle_timer = now;
                test_cycle_state = TEST_CYCLE_SETTLE;

                Test_Send("[CYCLE] IR detected, conveyor stop");
            }
            else if ((now - test_cycle_timer) >= TEST_IR_TIMEOUT_MS) {
                Test_CycleFail("[ERR] IR detect timeout");
            }
            break;


        /* 컨베이어 정지 후 200ms 정착 */
        case TEST_CYCLE_SETTLE:
            if ((now - test_cycle_timer) >= TEST_SETTLE_MS) {

                /*
                 * Z 포토센서가 이미 감지된 경우에는
                 * 하강하지 않고 현재 위치를 고정한다.
                 */
                if (Z_PhotoDetected()) {
                    Z_Enable(true);

                    test_cycle_timer = now;
                    test_cycle_state = TEST_CYCLE_Z_HOLD;

                    Test_Send("[CYCLE] Z already fixed");
                }
                else {
                    /*
                     * 현재 수동 명령 기준:
                     * +step = Z축 내려감
                     */
                    Z_MoveSteps(
                        +TEST_Z_DESCEND_STEPS,
                        TEST_CYCLE_Z_HZ
                    );

                    test_cycle_timer = now;
                    test_cycle_state = TEST_CYCLE_Z_DESCEND;

                    Test_Send("[CYCLE] Z descend start");
                }
            }
            break;


        /* Z축 하강 → 포토센서 감지 대기 */
        case TEST_CYCLE_Z_DESCEND:
            if (Z_PhotoDetected()) {
                /*
                 * 이동 펄스는 멈추고
                 * Z축 고정 토크는 유지
                 */
                Z_StopHold();

                test_cycle_timer = now;
                test_cycle_state = TEST_CYCLE_Z_HOLD;

                Test_Send("[CYCLE] Z fixed");
            }
            else if (!Z_IsBusy()) {
                /*
                 * 최대 이동량까지 갔는데
                 * 포토센서가 감지되지 않음
                 */
                Test_CycleFail("[ERR] Z photo not detected");
            }
            else if ((now - test_cycle_timer) >= TEST_Z_TIMEOUT_MS) {
                Test_CycleFail("[ERR] Z descend timeout");
            }
            break;


        /*
         * Z축 고정 상태에서 수확 시뮬레이션
         * Z_StopHold() 상태이므로 고정 토크 유지
         */
        case TEST_CYCLE_Z_HOLD:
            if ((now - test_cycle_timer) >= TEST_HARVEST_SIM_MS) {

                /*
                 * 현재 수동 명령 기준:
                 * -step = Z축 올라감
                 */
                Z_MoveSteps(
                    -TEST_Z_RETURN_STEPS,
                    TEST_CYCLE_Z_HZ
                );

                test_cycle_timer = now;
                test_cycle_state = TEST_CYCLE_Z_RETURN;

                Test_Send("[CYCLE] Z return start");
            }
            break;


        /* Z축 복귀 완료 대기 */
        case TEST_CYCLE_Z_RETURN:
            if (!Z_IsBusy()) {
                Z_Enable(false);

                Conveyor_Enable(true);
                Conveyor_SetDir(true);
                Conveyor_StartHz(TEST_CONV_HZ);

                test_cycle_timer = now;
                test_cycle_state = TEST_CYCLE_WAIT_EXIT;

                Test_Send("[CYCLE] Conveyor eject start");
            }
            else if (
                (now - test_cycle_timer) >=
                TEST_Z_RETURN_TIMEOUT_MS
            ) {
                Test_CycleFail("[ERR] Z return timeout");
            }
            break;


        /* 최소 500ms 구동 후 IR 해제 대기 */
        case TEST_CYCLE_WAIT_EXIT:
            if (
                (now - test_cycle_timer) >= TEST_EXIT_MIN_RUN_MS &&
                !IR_Detected()
            ) {
                Conveyor_Stop();
                Conveyor_Enable(false);

                test_cycle_state = TEST_CYCLE_IDLE;

                Test_Send("[CYCLE] DONE");
            }
            else if (
                (now - test_cycle_timer) >= TEST_EXIT_TIMEOUT_MS
            ) {
                Test_CycleFail("[ERR] Conveyor exit timeout");
            }
            break;


        default:
            Test_CycleFail("[ERR] Invalid cycle state");
            break;
    }
}
static void Test_Process(uint8_t *buf)
{
    // C1: 컨베이어 구동
	// C1: 사이클을 취소하고 컨베이어 수동 구동
	if (buf[0] == 'C' && buf[1] == '1') {
	    test_cycle_state = TEST_CYCLE_IDLE;

	    /* 진행 중이던 Z축 사이클도 정지 */
	    Z_Stop();

	    /* 컨베이어를 깨끗하게 재시작 */
	    Conveyor_Stop();
	    Conveyor_Enable(true);
	    Conveyor_SetDir(true);
	    Conveyor_StartHz(TEST_CONV_HZ);

	    Test_Send("[OK] Conveyor START");
	}
    // C0: 컨베이어 정지
    /* C0: 수동 정지뿐 아니라 사이클도 취소 */
	else if (buf[0] == 'C' && buf[1] == '0') {
	    test_cycle_state = TEST_CYCLE_IDLE;

	    Conveyor_Stop();
	    Conveyor_Enable(false);
	    Z_Stop();

	    Test_Send("[OK] Conveyor STOP");
	}
    /* CY: 두 번째 컨베이어 한 사이클 */
    else if (buf[0] == 'C' && buf[1] == 'Y') {
        Test_ConveyorCycleStart();

        /* 삭제해야 함:
           Test_Send("[OK] Conveyor ONE CYCLE");
        */
    }
    // ZF: Z축 CW (호밍 방향)
    else if (buf[0] == 'Z' && buf[1] == 'F') {
        Z_MoveSteps(+TEST_Z_STEPS, TEST_Z_HZ);
        Test_Send("[OK] Z CW (forward)");
    }
    // ZB: Z축 CCW (복귀 방향)
    else if (buf[0] == 'Z' && buf[1] == 'B') {
        Z_MoveSteps(-TEST_Z_STEPS, TEST_Z_HZ);
        Test_Send("[OK] Z CCW (back)");
    }
    // ZS: Z축 즉시 정지
    else if (buf[0] == 'Z' && buf[1] == 'S') {
        Z_Stop();
        Test_Send("[OK] Z STOP");
    }
    // IR: 적외선 센서 상태
    else if (buf[0] == 'I' && buf[1] == 'R') {
        if (IR_Detected())
            Test_Send("[IR] DETECTED");
        else
            Test_Send("[IR] not detected");
    }
    // LM: 리밋스위치 상태
    else if (buf[0] == 'L' && buf[1] == 'M') {
        if (Z_PhotoDetected())
            Test_Send("[PHOTO] HIT");
        else
            Test_Send("[PHOTO] not hit");
    }
    // E1: 긴급정지
    else if (buf[0] == 'E' && buf[1] == '1') {
        test_cycle_state = TEST_CYCLE_IDLE;

        Conveyor_Stop();
        Conveyor_Enable(false);
        Z_Stop();

        Test_Send("[ESTOP] ALL STOP");
    }
    // 알 수 없는 커맨드
    else {
        Test_Send("[ERR] unknown cmd");
    }
}

// ── UART 수신 콜백 (main.c에서 호출) ────────
void Test_RxCallback(void)
{
    if (test_rx_byte == '\n') {
        test_buf[test_buf_idx] = '\0';
        memcpy(test_cmd_buf, test_buf, TEST_BUF_SIZE);
        test_cmd_ready = true;
        test_buf_idx = 0;
    } else if (test_rx_byte == '\r') {
        // 무시
    } else if (test_buf_idx < TEST_BUF_SIZE - 1) {
        test_buf[test_buf_idx++] = test_rx_byte;
    } else {
        test_buf_idx = 0;
    }
    HAL_UART_Receive_IT(&huart2, &test_rx_byte, 1);
}

// ── 초기화 ───────────────────────────────────
void Test_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &test_rx_byte, 1);
    Test_Send("=== TEST MODE ===");
    Test_Send("C1  Conveyor START");
    Test_Send("C0  Conveyor STOP");
    Test_Send("ZF  Z CW  +5000step");
    Test_Send("ZB  Z CCW -5000step");
    Test_Send("ZS  Z STOP");
    Test_Send("IR  IR sensor state");
    Test_Send("LM  Photo sensor state");
    Test_Send("E1  ESTOP");
    Test_Send("=================");
}

// ── 메인 루프에서 호출 ───────────────────────
void Test_Run(void)
{
    if (test_cmd_ready) {
        test_cmd_ready = false;
        Test_Process(test_cmd_buf);
    }

    Test_ConveyorCycleRun();
}
