// cartesian.c
#include "cartesian.h"
#include "first_convey.h"
#include "sensor.h"
#include "board_pin.h"
#include <math.h>

extern TIM_HandleTypeDef  htim2;   //  직교로봇 x축 PWM: TIM2 CH1
extern TIM_HandleTypeDef htim5;   // 직교로봇 Z축 PWM
extern UART_HandleTypeDef huart2;  // uart인데 아직 용도 안정함

// 나중에 조정할 변수값들
// X축
#define X_BELT_PITCH_MM     2.0f    // GT2 벨트: 2mm
#define X_PULLEY_TEETH      20      // 풀리 이빨 수
#define X_MOTOR_STEPS       200     // 모터 1회전당 스텝 (1.8도 모터)
#define X_MICROSTEPS        4      // 마이크로스테핑 (드라이버 설정), 800

static float g_x_steps_per_mm = (X_MOTOR_STEPS * X_MICROSTEPS) / (X_BELT_PITCH_MM * X_PULLEY_TEETH); // X츅 펄스당이동 타이밍벨트 계산
static float g_x_ref_cm = 12.0f; // x축 위치보정 값, 이 거리까지 조정한다.
static uint32_t g_x_move_hz = 1000; // X축 이동 속도
static float g_deadband_mm = 1.0f; // 거리(x축)허용 오차 범위

#define X_DIR_INVERT   1   // 방향
#define X_EN_ACTIVE    0   // EN

// Z축
#define Z_LEAD_MM 8.0f
#define Z_MOTOR_STEPS 200
#define Z_MICROSTEPS 4 // 800
static float g_z_steps_per_mm = (Z_MOTOR_STEPS * Z_MICROSTEPS) / Z_LEAD_MM;
static float g_z_ref_cm = 12.0f;
static uint32_t g_z_move_hz = 4000; // z축 속도

#define Z_DIR_INVERT   1	// 방향
#define Z_EN_ACTIVE    0	// EN

// 직교로봇 홈(homing) 파라미터
static int32_t x_home_steps = 0;
static int32_t z_home_steps = 0;
static uint32_t g_x_homing_speed_hz   = 2300;  // X(벨트) SEEK 속도
static uint32_t g_z_homing_speed_hz   = 3000;   // Z(리드스크류) SEEK 속도
static float g_x_home_offset = 130.0f;   // 홈 160mm
static float g_z_home_offset = 50.0f;   // 홈 30mm
static float g_x_backoff_mm = 10.0f;	// 백오프 10mm
static float g_z_backoff_mm = 10.0f;

// 홈 포지션 상태
typedef enum {
    HOMING_IDLE = 0,
    HOMING_MOVE_TO_LIMIT,
    HOMING_BACKOFF,
    HOMING_WAIT_BACKOFF,
    HOMING_MOVE_TO_POS,
    HOMING_WAIT_MOVE,
    HOMING_DONE
} HomingState;
static HomingState homing_state = HOMING_IDLE;
static volatile bool x_limit_hit = false;
static volatile bool z_limit_hit = false;

typedef enum {
  AXIS_SEEK = 0,     // 리밋 찾는 중
  AXIS_BACKOFF,      // 백오프 이동 시작
  AXIS_WAIT_BACKOFF, // 백오프 완료 대기
  AXIS_MOVE_TO_POS,   // 홈 위치 이동
  AXIS_WAIT_MOVE,     // 홈 위치 이동 대기
  AXIS_DONE          // 완료
} AxisHomeState;

static AxisHomeState x_home = AXIS_SEEK;
static AxisHomeState z_home = AXIS_SEEK;

static int32_t x_backoff_steps = 0;
static int32_t z_backoff_steps = 0;

// 작업상태
typedef enum {
  ST_RUN_CONVEY = 0,   // 컨베이어 구동(IR 감지 전)
  ST_MEASURE_MOVE_X,   // 거리 측정 후 X 이동 시작
  ST_WAIT_X_DONE,      // X 이동 끝날 때까지 대기
  ST_DONE              // 완료(여기서 다음 파종 단계는 너가 나중에 추가)
} State;

static State state = ST_RUN_CONVEY;

static bool ir_latched = false;      // IR 감지 순간 1번만 전환
static bool busy = false;            // X축 이동 중인지
static volatile uint32_t x_remain = 0;
static volatile uint32_t z_remain = 0;
static volatile bool x_moving = false;
static volatile bool z_moving = false;

static uint32_t homing_start_time = 0;

//  TIM2 클럭 계산
// TIM2는 APB1 타이머
static uint32_t TIM2_GetClockHz(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2;
  return pclk1;
}

// TIM5
static uint32_t TIM5_GetClockHz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2;
    return pclk1;
}

//  STEP PWM 주파수 설정(TIM2 CH1, TIM5 CH1)
static void X_StepPWM_SetHz(uint32_t hz)
{
  if (hz < 1) hz = 1;

  uint32_t tim_clk = TIM2_GetClockHz();
  uint32_t arr = (tim_clk / hz) - 1;

  __HAL_TIM_SET_PRESCALER(&htim2, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr / 2); // 50%
  __HAL_TIM_SET_COUNTER(&htim2, 0);
}
static void Z_StepPWM_SetHz(uint32_t hz)
{
    if (hz < 1) hz = 1;
    uint32_t tim_clk = TIM5_GetClockHz();
    uint32_t arr = (tim_clk / hz) - 1;

    __HAL_TIM_SET_PRESCALER(&htim5, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim5, arr);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, arr / 2);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
}

//  방향설정
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

//  EN설정(허용)
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

// stop
static void X_Stop(void)
{
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
  X_Enable(false);
  x_remain = 0;
  x_moving = false;

}
static void Z_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_CC1);
    Z_Enable(false);
    z_remain = 0;
    z_moving = false;

}

// move
static void X_StartMoveSteps(int32_t steps, uint32_t step_hz)
{
  if (steps == 0) return;

  bool dir = (steps >= 0);
  uint32_t n = (uint32_t)(dir ? steps : -steps);

  X_SetDir(dir);
  X_Enable(true);

  x_remain = n;
  x_moving = true;


  X_StepPWM_SetHz(step_hz);  // 고정 속도!

  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

static void Z_StartMoveSteps(int32_t steps, uint32_t step_hz)
{
    if (steps == 0) return;

    bool dir = (steps >= 0);
    uint32_t n = (uint32_t)(dir ? steps : -steps);

    Z_SetDir(dir);
    Z_Enable(true);

    z_remain = n;
    z_moving = true;


    Z_StepPWM_SetHz(step_hz);  // 고정 속도!

    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC1);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC1);

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
}
void Cartesian_ResetSequence(void)
{
    // 혹시 남아있는 PWM/IT를 확실히 끊어줌 (중요)
    X_Stop();
    Z_Stop();

    // 상위 작업 시퀀스 초기화
    state = ST_RUN_CONVEY;
    ir_latched = false;

    busy = false;

    // 리밋 관련 플래그 정리
    x_limit_hit = false;
    z_limit_hit = false;
}
// 리밋 눌림 감지
void Cartesian_limit(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == X_LIM_PIN) {
    if (!x_limit_hit) {
      x_limit_hit = true;
      X_Stop();
      if (x_home == AXIS_SEEK) x_home = AXIS_BACKOFF;  //  X만 백오프 예약
    }
  }
  else if (GPIO_Pin == Z_LIM_PIN) {
    if (!z_limit_hit) {
      z_limit_hit = true;
      Z_Stop();
      if (z_home == AXIS_SEEK) z_home = AXIS_BACKOFF;  // Z만 백오프 예약
    }
  }
}
// 홈 찾기 시작
void Cartesian_StartHoming(void)
{
  homing_state = HOMING_MOVE_TO_LIMIT; // 큰 틀 상태는 유지해도 됨
  x_limit_hit = false;
  z_limit_hit = false;

  x_home = AXIS_SEEK;
  z_home = AXIS_SEEK;

  // 백오프 스텝 미리 계산
  x_backoff_steps = (int32_t)(g_x_backoff_mm * g_x_steps_per_mm);
  z_backoff_steps = (int32_t)(g_z_backoff_mm * g_z_steps_per_mm);

  // 홈 오프셋 스텝 계산
  x_home_steps = (int32_t)(g_x_home_offset * g_x_steps_per_mm);
  z_home_steps = (int32_t)(g_z_home_offset * g_z_steps_per_mm);

  // 이미 눌려있는 상태면 즉시 hit 처리 후 stop
  if (HAL_GPIO_ReadPin(X_LIM_PORT, X_LIM_PIN) == GPIO_PIN_RESET) {
    x_limit_hit = true;
    X_Stop();
    x_home = AXIS_BACKOFF; // 바로 백오프 단계로
  }
  if (HAL_GPIO_ReadPin(Z_LIM_PORT, Z_LIM_PIN) == GPIO_PIN_RESET) {
    z_limit_hit = true;
    Z_Stop();
    z_home = AXIS_BACKOFF;
  }

  homing_start_time = HAL_GetTick();
}

// 홈 찾기
void Cartesian_HomingTask(void)
{
  uint32_t now = HAL_GetTick();

  // ---------- X축 ----------
  if (x_home == AXIS_SEEK) {
    if (!x_moving && !x_limit_hit) {
      X_StartMoveSteps(-1000000, g_x_homing_speed_hz); // 리밋 방향
    }
    // x_limit_hit는 EXTI에서 true로 바뀜
  }
  else if (x_home == AXIS_BACKOFF) {
    if (!x_moving) {
      X_StartMoveSteps(+x_backoff_steps, g_x_homing_speed_hz); // 반대방향 백오프
      x_home = AXIS_WAIT_BACKOFF;
    }
  }
  else if (x_home == AXIS_WAIT_BACKOFF) {
    if (!x_moving) {
      x_home = AXIS_MOVE_TO_POS;
    }
  }
  else if (x_home == AXIS_MOVE_TO_POS) {
    if (!x_moving) {
      // 백오프 후 +방향으로 홈 오프셋 이동 (방향은 기구에 맞게 +/- 바꿔야 함)
      X_StartMoveSteps(+x_home_steps, g_x_homing_speed_hz);
      x_home = AXIS_WAIT_MOVE;
    }
  }
  else if (x_home == AXIS_WAIT_MOVE) {
    if (!x_moving) {
      x_home = AXIS_DONE;
    }
  }

  // ---------- Z축 ----------
  if (z_home == AXIS_SEEK) {
    if (!z_moving && !z_limit_hit) {
      Z_StartMoveSteps(-1000000, g_z_homing_speed_hz);
    }
  }
  else if (z_home == AXIS_BACKOFF) {
    if (!z_moving) {
      Z_StartMoveSteps(+z_backoff_steps, g_z_homing_speed_hz);
      z_home = AXIS_WAIT_BACKOFF;
    }
  }
  else if (z_home == AXIS_WAIT_BACKOFF) {
    if (!z_moving) {
      z_home = AXIS_MOVE_TO_POS;
    }
  }
  else if (z_home == AXIS_MOVE_TO_POS) {
    if (!z_moving) {
      Z_StartMoveSteps(+z_home_steps, g_z_homing_speed_hz);
      z_home = AXIS_WAIT_MOVE;
    }
  }
  else if (z_home == AXIS_WAIT_MOVE) {
    if (!z_moving) {
      z_home = AXIS_DONE;
    }
  }

  // ---------- 전체 완료 ----------
  if (x_home == AXIS_DONE && z_home == AXIS_DONE) {
    homing_state = HOMING_DONE;
    busy = false;
  }

  // ---------- 안전 타임아웃 ----------
  if (now - homing_start_time > 10000) {
    X_Stop();
    Z_Stop();
    x_home = AXIS_DONE;
    z_home = AXIS_DONE;
    homing_state = HOMING_DONE;
    busy = false;
  }
}

bool Cartesian_IsHomingDone(void)
{
    return (homing_state == HOMING_DONE);
}

void Cartesian_OnTimOcCallback(TIM_HandleTypeDef *htim)
{
    // X축
    if (htim->Instance == TIM2 && x_moving) {
        if (x_remain > 0) {
            x_remain--;
            if (x_remain == 0) {
                X_Stop();
                //busy  = false;
            }
        }
    }

    // Z축
    else if (htim->Instance == TIM5 && z_moving) {
        if (z_remain > 0) {
            z_remain--;
            if (z_remain == 0) {
                Z_Stop();
            }
        }
    }
}
//  Update 인터럽트마다 1스텝 감소(= PWM 1주기당 1펄스라고 가정)
void Cartesian_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
	(void)htim;
//	if (htim->Instance == TIM2 && x_moving) {
//	        if (x_remain > 0) {
//	            x_remain--;
//	            if (x_remain == 0) X_Stop();
//	        }
//	    }
//	else if (htim->Instance == TIM5 && z_moving) {
//        if (z_remain > 0) {
//            z_remain--;
//            if (z_remain == 0) Z_Stop();
//        }
//    }
}

// ===================== 거리 측정 후 X 이동
static void MeasureAndMoveX(void)
{
  // 거리센서 읽기
  uint16_t raw = Sensor_Distance_ReadRawAvg(16);
  float v  = (3.3f * raw) / 4095.0f;
  float cm = Sensor_Distance_VoltageToCm(v);

  // 기준값 대비 오차(mm)
  float err_mm = (cm - g_x_ref_cm) * 10.0f;

  // 오차가 작으면 X축 이동 안 함
  if (fabsf(err_mm) <= g_deadband_mm) {
    state = ST_DONE;
    return;
  }

  // mm -> steps
  int32_t steps = (int32_t)(err_mm * g_x_steps_per_mm);

  // X축 이동 시작
  X_StartMoveSteps(steps, g_x_move_hz);
  state = ST_WAIT_X_DONE;
}

// main에서 계속 호출하는 Task
void Cartesian_Task(void)
{
    switch (state)
    {
        case ST_RUN_CONVEY:
            FirstConvey_Task();

            if (FirstConvey_IsStopped() && !ir_latched) {
                ir_latched = true;
                state = ST_MEASURE_MOVE_X;
            }
            break;

        case ST_MEASURE_MOVE_X:
            MeasureAndMoveX();
            break;

        case ST_WAIT_X_DONE:
        	if (!x_moving) {
        	        busy  = false;
        	        state = ST_DONE;
        	    }
            // 타이머 콜백에서 ST_DONE으로 변경
            break;

        case ST_DONE:
            // Z축 파종 추가 예정
            break;

        default:
            state = ST_RUN_CONVEY;
            break;
    }
}

///////////////////////////////// 설정 함수
void Cartesian_SetXStepsPerMM(float steps_per_mm)
{
  if (steps_per_mm > 0.1f) g_x_steps_per_mm = steps_per_mm;
}

void Cartesian_SetXRefCm(float ref_cm)
{
  if (ref_cm > 0.1f) g_x_ref_cm = ref_cm;
}

bool Cartesian_IsBusy(void)
{
  return busy;
}
