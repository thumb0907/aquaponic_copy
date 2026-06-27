// cartesian.c
#include "cartesian.h"
#include "first_convey.h"
#include "sensor.h"
#include "board_pin.h"
#include <stdbool.h>

extern TIM_HandleTypeDef htim2;   // 직교로봇 X축 PWM: TIM2 CH1
extern TIM_HandleTypeDef htim5;   // 직교로봇 Z축 PWM: TIM5 CH1

// ── X축 파라미터 ──────────────────────────────
#define X_BELT_PITCH_MM  2.0f
#define X_PULLEY_TEETH   20
#define X_MOTOR_STEPS    200
#define X_MICROSTEPS     8        // 1600 pulse/rev
static float    g_x_steps_per_mm = (X_MOTOR_STEPS * X_MICROSTEPS) / (X_BELT_PITCH_MM * X_PULLEY_TEETH);
static uint32_t g_x_move_hz      = 2800;   // X축 이동 속도 (실측 후 조정)
#define X_DIR_INVERT  0
#define X_EN_ACTIVE   1

// ── Z축 파라미터 ──────────────────────────────
#define Z_LEAD_MM      8.0f
#define Z_MOTOR_STEPS  200
#define Z_MICROSTEPS   8          // 1600 pulse/rev
static float g_z_steps_per_mm = (Z_MOTOR_STEPS * Z_MICROSTEPS) / Z_LEAD_MM;
#define Z_DIR_INVERT  1
#define Z_EN_ACTIVE   1

// Z축 가감속 파라미터
static uint32_t z_total_steps   = 0;
static uint32_t z_stepped       = 0;
static uint32_t g_z_accel_steps = 500;
static uint32_t z_prof_start_hz = 3500;
static uint32_t z_prof_max_hz   = 10000;

// ── 호밍 파라미터 ─────────────────────────────
static int32_t  x_home_steps         = 0;
static int32_t  z_home_steps         = 0;
static uint32_t g_x_homing_speed_hz  = 2800;
static uint32_t g_z_homing_speed_hz  = 10000;
static float    g_x_home_offset      = 125.0f;  // 백오프포함, 대기위치=140mm
static float    g_z_home_offset      = 58.0f;
static float    g_x_backoff_mm       = 15.0f;
static float    g_z_backoff_mm       = 2.0f;
static int32_t  x_backoff_steps      = 0;
static int32_t  z_backoff_steps      = 0;

// ── 픽업 파라미터 ─────────────────────────────
static float    g_pick_z_down_mm  = 110.0f;   // 씨앗통에서 Z 하강 거리 (실측 후 조정)
static float    g_pick_x_sow_mm   = 250.0f;  // 씨앗통 → 파종 위치 X 이동 거리 (실측 후 조정)
static uint32_t g_pump_on_ms      = 5000;    // 펌프 ON 유지 시간 (ms), PU 테스트 명령에서만 사용하는 펌프 단독 구동 시간

// ── 파종 파라미터 ─────────────────────────────
#define SOW_ROWS 6
static float g_sow_z_down_mm = 80.0f;   // 파종 Z 하강 거리 (실측 후 조정) - 80이 최대하강거리
static const float sow_row_feed_mm[SOW_ROWS - 1] = {
    35.0f,
    35.0f,
    150.0f,  // 트레이와 트레이 사이 거리
    35.0f,
    35.0f,
};
static uint8_t sow_row = 0;

// ── 호밍 상태 ─────────────────────────────────
typedef enum {
    HOMING_IDLE = 0,
    HOMING_MOVE_TO_LIMIT,
    HOMING_DONE,
    HOMING_ERROR
} HomingState;
static HomingState homing_state   = HOMING_IDLE;
static uint32_t    homing_start_time = 0;

typedef enum {
    AXIS_SEEK = 0,
    AXIS_BACKOFF,
    AXIS_WAIT_BACKOFF,
    AXIS_MOVE_TO_POS,
    AXIS_WAIT_MOVE,
    AXIS_DONE
} AxisHomeState;
static AxisHomeState x_home = AXIS_SEEK;
static AxisHomeState z_home = AXIS_SEEK;

static volatile bool x_limit_hit      = false;  // 호밍용
static volatile bool z_limit_hit      = false;  // 호밍용
static volatile bool x_pick_limit_hit = false;  // 픽업용 리밋 플래그

// ── 작업 상태 ─────────────────────────────────
typedef enum {
    ST_RUN_CONVEY = 0,

    // 픽업 시퀀스
    ST_PICK_MOVE_X,       // X축 리밋 방향으로 이동
    ST_PICK_WAIT_X,       // 리밋 감지 대기 + 백오프
    ST_PICK_BACKOFF,      // 백오프 이동 중 + 완료 대기
    ST_PICK_Z_DOWN,       // Z 30mm 하강 (씨앗통으로)
    ST_PICK_WAIT_Z_DOWN,  // Z 하강 완료 대기
    ST_PICK_PUMP_ON,      // 펌프 ON (씨앗 흡착)
    ST_PICK_Z_UP,         // Z 30mm 상승
    ST_PICK_WAIT_Z_UP,    // Z 상승 완료 대기
	ST_MOVE_TO_SOW_X,     // X +240mm 이동 (파종 위치)
    ST_PICK_WAIT_RET,     // X 이동 완료 대기

    // 파종 시퀀스
    ST_SOW_Z_DOWN,        // Z 40mm 하강
    ST_SOW_WAIT_Z_DOWN,   // Z 하강 완료 → 펌프 OFF (파종)
    ST_SOW_Z_UP,          // Z 40mm 상승
    ST_SOW_WAIT_Z_UP,     // Z 상승 완료 대기

    ST_SOW_WAIT_ROW,      // 컨베이어 다음 행 이동 완료 대기
    ST_EXIT_CONVEY,       // 배출 (IR 미감지까지)
    ST_DONE
} State;
static State state = ST_RUN_CONVEY;

// 테스트용 플래그
static bool g_test_pick_only   = false;
static bool g_test_picksow_one = false;

static bool     ir_latched = false;
static volatile uint32_t x_remain = 0;
static volatile uint32_t z_remain = 0;
static volatile bool     x_moving = false;
static volatile bool     z_moving = false;

// ── 타이머 클럭 ───────────────────────────────
static uint32_t TIM2_GetClockHz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2;
    return pclk1;
}
static uint32_t TIM5_GetClockHz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2;
    return pclk1;
}

// ── PWM 주파수 설정 ───────────────────────────
static void X_StepPWM_SetHz(uint32_t hz)
{
    if (hz < 1) hz = 1;
    uint32_t tim_clk = TIM2_GetClockHz();
    uint64_t arr64 = (tim_clk / hz) - 1ULL;
    if (arr64 < 2ULL)           arr64 = 2ULL;
    if (arr64 > 0xFFFFFFFFULL)  arr64 = 0xFFFFFFFFULL;
    uint32_t arr = (uint32_t)arr64;
    __HAL_TIM_SET_PRESCALER(&htim2, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr / 2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}
static void Z_StepPWM_SetHz(uint32_t hz)
{
    if (hz < 1) hz = 1;
    uint32_t tim_clk = TIM5_GetClockHz();
    uint64_t arr64 = (tim_clk / hz) - 1ULL;
    if (arr64 < 2ULL)           arr64 = 2ULL;
    if (arr64 > 0xFFFFFFFFULL)  arr64 = 0xFFFFFFFFULL;
    uint32_t arr = (uint32_t)arr64;
    __HAL_TIM_SET_PRESCALER(&htim5, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim5, arr);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, arr / 2);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
}

// ── 방향 / EN ────────────────────────────────
static void X_SetDir(bool dir)
{
    if (X_DIR_INVERT) dir = !dir;
    HAL_GPIO_WritePin(CAR_X_DIR_PORT, CAR_X_DIR_PIN,
                      dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void Z_SetDir(bool dir)
{
    if (Z_DIR_INVERT) dir = !dir;
    HAL_GPIO_WritePin(CAR_Z_DIR_PORT, CAR_Z_DIR_PIN,
                      dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void X_Enable(bool en)
{
#if X_EN_ACTIVE
    HAL_GPIO_WritePin(CAR_X_EN_PORT, CAR_X_EN_PIN,
                      en ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(CAR_X_EN_PORT, CAR_X_EN_PIN,
                      en ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}
static void Z_Enable(bool en)
{
#if Z_EN_ACTIVE
    HAL_GPIO_WritePin(CAR_Z_EN_PORT, CAR_Z_EN_PIN,
                      en ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(CAR_Z_EN_PORT, CAR_Z_EN_PIN,
                      en ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}
static void Pump_Set(bool on)
{
    HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ── 정지 ─────────────────────────────────────
static void X_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    X_Enable(false);
    x_remain = 0;
    x_moving = false;
}
static void Z_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
    Z_Enable(false);
    z_remain      = 0;
    z_moving      = false;
    z_total_steps = 0;
    z_stepped     = 0;
}

// ── 이동 ─────────────────────────────────────
static void X_StartMoveSteps(int32_t steps, uint32_t step_hz)
{
    if (steps == 0) return;
    bool dir = (steps >= 0);
    uint32_t n = (uint32_t)(dir ? steps : -steps);
    X_SetDir(dir);
    X_Enable(true);
    x_remain = n;
    x_moving = true;
    X_StepPWM_SetHz(step_hz);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
static void Z_MoveFixed(int32_t steps, uint32_t hz)  // 호밍/백오프용 고정속도
{
    if (steps == 0) return;
    bool dir = (steps >= 0);
    uint32_t n = (uint32_t)(dir ? steps : -steps);
    Z_SetDir(dir);
    Z_Enable(true);
    z_total_steps   = 0;   // 고정속도 모드
    z_stepped       = 0;
    z_prof_start_hz = hz;
    z_prof_max_hz   = hz;
    z_remain        = n;
    z_moving        = true;
    Z_StepPWM_SetHz(hz);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
}
static void Z_MoveProfile(int32_t steps, uint32_t start_hz, uint32_t max_hz)  // 가감속
{
    if (steps == 0) return;
    if (start_hz < 1)        start_hz = 1;
    if (max_hz < start_hz)   max_hz   = start_hz;
    bool dir = (steps >= 0);
    uint32_t n = (uint32_t)(dir ? steps : -steps);
    Z_SetDir(dir);
    Z_Enable(true);
    z_total_steps   = n;
    z_stepped       = 0;
    z_prof_start_hz = start_hz;
    z_prof_max_hz   = max_hz;
    z_remain        = n;
    z_moving        = true;
    Z_StepPWM_SetHz(z_prof_start_hz);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
}

// ── 리셋 ─────────────────────────────────────
void Cartesian_ResetSequence(void)
{
    X_Stop();
    Z_Stop();
    Pump_Set(false);

    state             = ST_RUN_CONVEY;
    ir_latched        = false;
    sow_row           = 0;
    x_limit_hit       = false;
    z_limit_hit       = false;
    x_pick_limit_hit  = false;

    x_home       = AXIS_SEEK;
    z_home       = AXIS_SEEK;
    homing_state = HOMING_IDLE;
}

// ── 리밋 감지 콜백 (EXTI에서 호출) ───────────
void Cartesian_limit(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == X_LIM_PIN) {
        if (HAL_GPIO_ReadPin(X_LIM_PORT, X_LIM_PIN) != GPIO_PIN_RESET) return;

        // PK/PW 중 X 리밋 감지
        if ((state == ST_PICK_MOVE_X || state == ST_PICK_WAIT_X) && !x_pick_limit_hit) {
            x_pick_limit_hit = true;
            X_Stop();
            return;
        }

        // 홈잉 중 X 리밋 감지
        if (homing_state == HOMING_MOVE_TO_LIMIT &&
            x_home == AXIS_SEEK &&
            !x_limit_hit) {
            x_limit_hit = true;
            X_Stop();
            x_home = AXIS_BACKOFF;
            return;
        }
    }

    if (GPIO_Pin == Z_LIM_PIN) {
        if (HAL_GPIO_ReadPin(Z_LIM_PORT, Z_LIM_PIN) != GPIO_PIN_RESET) return;

        if (homing_state == HOMING_MOVE_TO_LIMIT &&
            z_home == AXIS_SEEK &&
            !z_limit_hit) {
            z_limit_hit = true;
            Z_Stop();
            z_home = AXIS_BACKOFF;
        }
    }
}

// ── 호밍 시작 ────────────────────────────────
void Cartesian_StartHoming(void)
{
    homing_state = HOMING_MOVE_TO_LIMIT;
    x_limit_hit  = false;
    z_limit_hit  = false;
    x_home       = AXIS_SEEK;
    z_home       = AXIS_SEEK;

    x_backoff_steps = (int32_t)(g_x_backoff_mm * g_x_steps_per_mm);
    z_backoff_steps = (int32_t)(g_z_backoff_mm * g_z_steps_per_mm);
    x_home_steps    = (int32_t)(g_x_home_offset * g_x_steps_per_mm);
    z_home_steps    = (int32_t)(g_z_home_offset * g_z_steps_per_mm);

    // 이미 리밋 눌려있으면 즉시 백오프
    if (HAL_GPIO_ReadPin(X_LIM_PORT, X_LIM_PIN) == GPIO_PIN_RESET) {
        x_limit_hit = true;
        X_Stop();
        x_home = AXIS_BACKOFF;
    }
    if (HAL_GPIO_ReadPin(Z_LIM_PORT, Z_LIM_PIN) == GPIO_PIN_RESET) {
        z_limit_hit = true;
        Z_Stop();
        z_home = AXIS_BACKOFF;
    }

    homing_start_time = HAL_GetTick();
}

// ── 호밍 태스크 ──────────────────────────────
void Cartesian_HomingTask(void)
{
    uint32_t now = HAL_GetTick();

    if (homing_state == HOMING_ERROR || homing_state == HOMING_DONE) return;

    // X축
    if (x_home == AXIS_SEEK) {
        if (!x_moving && !x_limit_hit)
            X_StartMoveSteps(-1000000, g_x_homing_speed_hz);
    }
    else if (x_home == AXIS_BACKOFF) {
        if (!x_moving) {
            X_StartMoveSteps(+x_backoff_steps, g_x_homing_speed_hz);
            x_home = AXIS_WAIT_BACKOFF;
        }
    }
    else if (x_home == AXIS_WAIT_BACKOFF) {
        if (!x_moving) x_home = AXIS_MOVE_TO_POS;
    }
    else if (x_home == AXIS_MOVE_TO_POS) {
        if (!x_moving) {
            X_StartMoveSteps(+x_home_steps, g_x_homing_speed_hz);
            x_home = AXIS_WAIT_MOVE;
        }
    }
    else if (x_home == AXIS_WAIT_MOVE) {
        if (!x_moving) x_home = AXIS_DONE;
    }

    // Z축
    if (z_home == AXIS_SEEK) {
        if (!z_moving && !z_limit_hit)
        	Z_MoveProfile(-1000000, 3000, g_z_homing_speed_hz);
    }
    else if (z_home == AXIS_BACKOFF) {
        if (!z_moving) {
            Z_MoveFixed(+z_backoff_steps, 3000);
            z_home = AXIS_WAIT_BACKOFF;
        }
    }
    else if (z_home == AXIS_WAIT_BACKOFF) {
        if (!z_moving) z_home = AXIS_MOVE_TO_POS;
    }
    else if (z_home == AXIS_MOVE_TO_POS) {
        if (!z_moving) {
            Z_MoveProfile(+z_home_steps, 3000, g_z_homing_speed_hz);
            z_home = AXIS_WAIT_MOVE;
        }
    }
    else if (z_home == AXIS_WAIT_MOVE) {
        if (!z_moving) z_home = AXIS_DONE;
    }

    // 완료
    if (x_home == AXIS_DONE && z_home == AXIS_DONE) {
        homing_state = HOMING_DONE;
    }

    // 타임아웃 15초
    if (now - homing_start_time > 15000) {
        X_Stop();
        Z_Stop();
        homing_state = HOMING_ERROR;
    }
}

// ── 메인 태스크(전체 사이클) ──────────────────────────────
void Cartesian_Task(void)
{
    switch (state)
    {
        // 컨베이어 구동 → IR 감지 대기
        case ST_RUN_CONVEY:
            FirstConvey_Task();
            if (FirstConvey_IsStopped() && !ir_latched) {
                ir_latched = true;
                state = ST_PICK_MOVE_X;  // IR 감지되면 바로 픽업 시작
            }
            break;

        // X축 리밋 방향으로 이동 (리밋 칠 때까지)
        case ST_PICK_MOVE_X:
            x_pick_limit_hit = false;
            X_StartMoveSteps(-1000000, g_x_move_hz);  // 리밋 방향
            state = ST_PICK_WAIT_X;
            break;

        // 리밋 감지 대기 → 감지되면 백오프 시작
        case ST_PICK_WAIT_X:
            if (x_pick_limit_hit && !x_moving) {
                // 리밋 확인 후 백오프(+ g_x_backoff_mm)
                X_StartMoveSteps(+(int32_t)(g_x_backoff_mm * g_x_steps_per_mm), g_x_move_hz);
                state = ST_PICK_BACKOFF;
            }
            break;

        // 백오프 완료 대기
        case ST_PICK_BACKOFF:
            if (!x_moving) state = ST_PICK_Z_DOWN;
            break;

        // Z축 90mm 하강 (씨앗통으로)
        case ST_PICK_Z_DOWN:
        	HAL_Delay(200);
            Z_MoveProfile(+(int32_t)(g_pick_z_down_mm * g_z_steps_per_mm), 3500, 8000);
            state = ST_PICK_WAIT_Z_DOWN;
            break;

        // Z 하강 완료 → 펌프 ON
        case ST_PICK_WAIT_Z_DOWN:

            if (!z_moving) state = ST_PICK_PUMP_ON;
            break;

        // 펌프 ON (씨앗 흡착 시작)
        case ST_PICK_PUMP_ON:
        	HAL_Delay(200);
            Pump_Set(true);
            HAL_Delay(1500);
            state = ST_PICK_Z_UP;
            break;

        // Z축 90mm 상승
        case ST_PICK_Z_UP:
            Z_MoveProfile(-(int32_t)(g_pick_z_down_mm * g_z_steps_per_mm), 3500, 8000);
            state = ST_PICK_WAIT_Z_UP;
            break;

        // Z 상승 완료 → X +240mm 이동 (파종 위치)
        case ST_PICK_WAIT_Z_UP:
            if (!z_moving) {
                if (g_test_pick_only) {
                    // 픽업 테스트: Z 올리고 펌프 OFF → 완료
                    Pump_Set(false);
                    g_test_pick_only = false;
                    state = ST_DONE;
                } else {
                    state = ST_MOVE_TO_SOW_X;
                }
            }
            break;

        // X +240mm 이동 (씨앗통 → 파종 위치)
        case ST_MOVE_TO_SOW_X:
            X_StartMoveSteps(+(int32_t)(g_pick_x_sow_mm * g_x_steps_per_mm), g_x_move_hz);
            state = ST_PICK_WAIT_RET;
            break;

        // X 이동 완료 대기
        case ST_PICK_WAIT_RET:
            if (!x_moving) state = ST_SOW_Z_DOWN;
            break;

        // Z 40mm 하강 (파종)
        case ST_SOW_Z_DOWN:
            Z_MoveProfile(+(int32_t)(g_sow_z_down_mm * g_z_steps_per_mm), 3500, 9000);
            state = ST_SOW_WAIT_Z_DOWN;
            break;

        // Z 하강 완료 → 펌프 OFF (씨앗 놓기 = 파종)
        case ST_SOW_WAIT_Z_DOWN:
            if (!z_moving) {
            	HAL_Delay(500);
                Pump_Set(false);
                state = ST_SOW_Z_UP;
            }
            break;

        // Z 40mm 상승
        case ST_SOW_Z_UP:
            Z_MoveProfile(-(int32_t)(g_sow_z_down_mm * g_z_steps_per_mm), 3500, 9000);
            state = ST_SOW_WAIT_Z_UP;
            break;

        // Z 상승 완료 → 다음 행 or 배출
        case ST_SOW_WAIT_Z_UP:
            if (!z_moving) {
                sow_row++;
                if (g_test_picksow_one) {
                    g_test_picksow_one = false;
                    state = ST_DONE;
                } else if (sow_row < SOW_ROWS) {
                    // 다음 행: X를 씨앗통으로 돌아가서 다시 픽업
                    FirstConvey_MoveDistance(sow_row_feed_mm[sow_row - 1]);
                    state = ST_SOW_WAIT_ROW;
                } else {
                    state = ST_EXIT_CONVEY;
                }
            }
            break;

        // 컨베이어 행 이동 완료 대기 → 픽업 다시 시작
        case ST_SOW_WAIT_ROW:
            if (FirstConvey_IsMoveDone()) state = ST_PICK_MOVE_X;
            break;

        // 배출: IR 미감지까지 컨베이어 구동
        case ST_EXIT_CONVEY:
            if (!Sensor_IR_Detected()) {
                FirstConvey_ForceStop();
                state = ST_DONE;
            } else {
                // 멈춰있으면 10mm씩 전진
                if (FirstConvey_IsMoveDone())
                    FirstConvey_MoveDistance(10.0f);
            }
            break;

        case ST_DONE:
            break;

        default:
            state = ST_RUN_CONVEY;
            break;
    }
}

// ── 인터럽트 콜백 ────────────────────────────
void Cartesian_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
    // X축 스텝 카운트
    if (htim->Instance == TIM2 && x_moving) {
        if (x_remain > 0) {
            x_remain--;
            if (x_remain == 0) X_Stop();
        }
    }
    // Z축 스텝 카운트 + 가감속
    else if (htim->Instance == TIM5 && z_moving) {
        if (z_remain > 0) {
            z_remain--;

            if (z_total_steps > 0) {  // 가감속 모드
                z_stepped++;
                uint32_t accel = g_z_accel_steps;
                if (accel > z_total_steps) accel = z_total_steps;

                uint32_t new_hz;
                uint32_t decel_start = (z_total_steps > accel) ? (z_total_steps - accel) : 0;

                if (z_stepped < accel) {
                    new_hz = z_prof_start_hz
                           + (z_prof_max_hz - z_prof_start_hz) * z_stepped / accel;
                } else if (z_stepped >= decel_start) {
                    uint32_t steps_left = z_remain;
                    if (steps_left < accel) {
                        new_hz = z_prof_start_hz
                               + (z_prof_max_hz - z_prof_start_hz) * steps_left / accel;
                    } else {
                        new_hz = z_prof_max_hz;
                    }
                } else {
                    new_hz = z_prof_max_hz;
                }

                if (new_hz < z_prof_start_hz) new_hz = z_prof_start_hz;
                Z_StepPWM_SetHz(new_hz);
            }

            if (z_remain == 0) Z_Stop();
        }
    }
}

// ── 상태 조회 ────────────────────────────────
bool Cartesian_IsHomingStarted(void) { return (homing_state != HOMING_IDLE); }
bool Cartesian_IsHomingDone(void)    { return (homing_state == HOMING_DONE); }
bool Cartesian_IsHomingError(void)   { return (homing_state == HOMING_ERROR); }
bool Cartesian_IsCycleDone(void)     { return (state == ST_DONE); }

CartesianStage Cartesian_GetStage(void)
{
    switch (state) {
        case ST_RUN_CONVEY:
            return CART_STAGE_RUN_CONVEY;

        case ST_PICK_MOVE_X:
        case ST_PICK_WAIT_X:
        case ST_PICK_BACKOFF:
        case ST_PICK_Z_DOWN:
        case ST_PICK_WAIT_Z_DOWN:
        case ST_PICK_PUMP_ON:
        case ST_PICK_Z_UP:
        case ST_PICK_WAIT_Z_UP:
        case ST_MOVE_TO_SOW_X:
        case ST_PICK_WAIT_RET:
            return CART_STAGE_PICKING;

        case ST_SOW_Z_DOWN:
        case ST_SOW_WAIT_Z_DOWN:
        case ST_SOW_Z_UP:
        case ST_SOW_WAIT_Z_UP:
        case ST_SOW_WAIT_ROW:
            return CART_STAGE_SEEDING;

        case ST_EXIT_CONVEY:
            return CART_STAGE_EJECTING;

        case ST_DONE:
            return CART_STAGE_DONE;

        default:
            return CART_STAGE_UNKNOWN;
    }
}

// ── 설정 함수 ────────────────────────────────
void Cartesian_SetXStepsPerMM(float steps_per_mm)
{
    if (steps_per_mm > 0.1f) g_x_steps_per_mm = steps_per_mm;
}

// ── 테스트용 함수 ────────────────────────────
void Cartesian_TestMoveX(int32_t steps, uint32_t hz)  { X_StartMoveSteps(steps, hz); }
void Cartesian_TestMoveZ(int32_t steps, uint32_t hz)  { Z_MoveProfile(steps, 800, hz); }
void Cartesian_TestStopX(void)                         { X_Stop(); }
void Cartesian_TestStopZ(void)                         { Z_Stop(); }

void Cartesian_TestPump(void)
{
    Pump_Set(true);
    HAL_Delay(g_pump_on_ms);
    Pump_Set(false);
}

void Cartesian_TestPickCycle(void)
{
    g_test_pick_only = true;
    x_pick_limit_hit = false;
    state = ST_PICK_MOVE_X;
}

void Cartesian_TestPickSowCycle(void)
{
    g_test_pick_only   = false;
    g_test_picksow_one = true;
    x_pick_limit_hit   = false;
    sow_row            = 0;
    state              = ST_PICK_MOVE_X;
}
