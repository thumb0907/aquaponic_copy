// cartesian.c
#include "cartesian.h"
#include "first_convey.h"
#include "sensor.h"
#include "board_pin.h"
#include <math.h>

extern TIM_HandleTypeDef  htim2;   //  직교로봇 x축 PWM: TIM2 CH1
extern TIM_HandleTypeDef htim5;   // 직교로봇 Z축 PWM

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

// Z축 가감속 파라미터
static uint32_t g_z_start_hz  = 500;   // 시작(최저) 속도
static uint32_t g_z_max_hz    = 5500;  // 최고 속도
static uint32_t g_z_accel_steps = 200; // 가속에 걸리는 스텝 수

// Z축 가감속 상태
static uint32_t z_total_steps   = 0;
static uint32_t z_stepped        = 0;   // 지금까지 한 스텝 수

// 직교로봇 홈(homing) 파라미터
static int32_t x_home_steps = 0;
static int32_t z_home_steps = 0;
static uint32_t g_x_homing_speed_hz   = 2000;  // X(벨트) SEEK 속도
static uint32_t g_z_homing_speed_hz   = 3000;   // Z(리드스크류) SEEK 속도
static float g_x_home_offset = 140.0f;   // 홈 140mm
static float g_z_home_offset = 60.0f;   // 홈 60mm

// 파종
#define SOW_COLS 5
#define SOW_ROWS 8
static float g_sow_x_pitch_mm = 40.0f;  // X칸 간격 (실측 후 조정)
static float g_sow_z_down_mm  = 30.0f;  // Z 하강 거리 (실측 후 조정)

static const float sow_row_feed_mm[SOW_ROWS - 1] = {
    40.0f,
    40.0f,
    40.0f,
    140.0f,  // 트레이 사이 거리
    40.0f,
    40.0f,
    40.0f,
};
static uint8_t sow_col = 0;
static uint8_t sow_row = 0;

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

// 작업상태
typedef enum {
    ST_RUN_CONVEY = 0,
    ST_MEASURE_MOVE_X,
    ST_WAIT_X_DONE,

    ST_SOW_Z_DOWN,        // Z 하강
    ST_SOW_WAIT_Z_DOWN,
    ST_SOW_Z_UP,          // Z 상승
    ST_SOW_WAIT_Z_UP,

    ST_SOW_NEXT_X,        // 다음 X칸
    ST_SOW_WAIT_X,

    ST_SOW_RETURN_X,      // X 원점 복귀
    ST_SOW_WAIT_RETURN_X,

    ST_SOW_NEXT_ROW,      // 컨베이어 다음 행
    ST_SOW_WAIT_ROW,

    ST_EXIT_CONVEY,       // IR 미감지까지 배출
    ST_DONE
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
    z_remain = 0;
    z_moving = false;

    z_total_steps = 0;  // 가감속
    z_stepped     = 0;  // 가감속
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

  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

static void Z_StartMoveSteps(int32_t steps, uint32_t step_hz)
{
	// 그냥 이동
	/*{
	if (steps == 0) return;

    bool dir = (steps >= 0);
    uint32_t n = (uint32_t)(dir ? steps : -steps);

    Z_SetDir(dir);
    Z_Enable(true);

    z_remain = n;
    z_moving = true;


    Z_StepPWM_SetHz(step_hz);  // 고정 속도!

    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);

    __HAL_TIM_SET_COUNTER(&htim5, 0);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	}*/

	// 가감속
	if (steps == 0) return;

	    bool dir = (steps >= 0);
	    uint32_t n = (uint32_t)(dir ? steps : -steps);

	    Z_SetDir(dir);
	    Z_Enable(true);

	    z_total_steps = n;
	    z_remain      = n;
	    z_stepped     = 0;
	    z_moving      = true;

	    // 시작은 저속으로
	    Z_StepPWM_SetHz(g_z_start_hz);

	    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
	    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
	    __HAL_TIM_SET_COUNTER(&htim5, 0);
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
    sow_col    = 0;
    sow_row    = 0;
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
      //if (x_home == AXIS_SEEK) x_home = AXIS_BACKOFF;  //  X만 백오프 예약
      if (x_home == AXIS_SEEK) x_home = AXIS_MOVE_TO_POS;
    }
  }
  else if (GPIO_Pin == Z_LIM_PIN) {
    if (!z_limit_hit) {
      z_limit_hit = true;
      Z_Stop();
      //if (z_home == AXIS_SEEK) z_home = AXIS_BACKOFF;  // Z만 백오프 예약
      if (z_home == AXIS_SEEK) z_home = AXIS_MOVE_TO_POS;
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
  //x_backoff_steps = (int32_t)(g_x_backoff_mm * g_x_steps_per_mm);
  //z_backoff_steps = (int32_t)(g_z_backoff_mm * g_z_steps_per_mm);

  // 홈 오프셋 스텝 계산
  x_home_steps = (int32_t)(g_x_home_offset * g_x_steps_per_mm);
  z_home_steps = (int32_t)(g_z_home_offset * g_z_steps_per_mm);

  // 이미 눌려있는 상태면 즉시 hit 처리 후 stop
  if (HAL_GPIO_ReadPin(X_LIM_PORT, X_LIM_PIN) == GPIO_PIN_RESET) {
    x_limit_hit = true;
    X_Stop();
    //x_home = AXIS_BACKOFF; // 바로 백오프 단계로
    x_home = AXIS_MOVE_TO_POS;
  }
  if (HAL_GPIO_ReadPin(Z_LIM_PORT, Z_LIM_PIN) == GPIO_PIN_RESET) {
    z_limit_hit = true;
    Z_Stop();
    //z_home = AXIS_BACKOFF;
    z_home = AXIS_MOVE_TO_POS;
  }

  homing_start_time = HAL_GetTick();
}

// 홈 찾기
void Cartesian_HomingTask(void)
{
  uint32_t now = HAL_GetTick();

  // X축
  if (x_home == AXIS_SEEK) {
    if (!x_moving && !x_limit_hit) {
      X_StartMoveSteps(-1000000, g_x_homing_speed_hz); // 리밋 방향
    }
    // x_limit_hit는 EXTI에서 true로 바뀜
  }
  /*else if (x_home == AXIS_BACKOFF) {
    if (!x_moving) {
      X_StartMoveSteps(+x_backoff_steps, g_x_homing_speed_hz); // 반대방향 백오프
      x_home = AXIS_WAIT_BACKOFF;
    }
  }
  else if (x_home == AXIS_WAIT_BACKOFF) {
    if (!x_moving) {
      x_home = AXIS_MOVE_TO_POS;
    }
  }*/
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

  //  Z축
  if (z_home == AXIS_SEEK) {
    if (!z_moving && !z_limit_hit) {
      Z_StartMoveSteps(-1000000, g_z_homing_speed_hz);
    }
  }
  /*else if (z_home == AXIS_BACKOFF) {
    if (!z_moving) {
      Z_StartMoveSteps(+z_backoff_steps, g_z_homing_speed_hz);
      z_home = AXIS_WAIT_BACKOFF;
    }
  }
  else if (z_home == AXIS_WAIT_BACKOFF) {
    if (!z_moving) {
      z_home = AXIS_MOVE_TO_POS;
    }
  }*/
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

  // 전체 완료
  if (x_home == AXIS_DONE && z_home == AXIS_DONE) {
    homing_state = HOMING_DONE;
    busy = false;
  }

  // 안전 타임아웃 (15초)
  if (now - homing_start_time > 15000) {
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

//void Cartesian_OnTimOcCallback(TIM_HandleTypeDef *htim) // cc인터럽트
//{
//    // X축
//    if (htim->Instance == TIM2 && x_moving) {
//        if (x_remain > 0) {
//            x_remain--;
//            if (x_remain == 0) {
//                X_Stop();
//                //busy  = false;
//            }
//        }
//    }
//
//    // Z축
//    else if (htim->Instance == TIM5 && z_moving) {
//        if (z_remain > 0) {
//            z_remain--;
//            if (z_remain == 0) {
//                Z_Stop();
//            }
//        }
//    }
//}
//  Update 인터럽트마다 1스텝 감소(= PWM 1주기당 1펄스라고 가정)
void Cartesian_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2 && x_moving) {
	        if (x_remain > 0) {
	            x_remain--;
	            if (x_remain == 0) X_Stop();
	        }
	    }
	/*else if (htim->Instance == TIM5 && z_moving) {
        if (z_remain > 0) {
            z_remain--;
            if (z_remain == 0) Z_Stop();
        }
    }*/
	else if (htim->Instance == TIM5 && z_moving) {
	        if (z_remain > 0) {
	            z_stepped++;
	            z_remain--;

	            // 가감속 Hz 계산
	            uint32_t accel = g_z_accel_steps;
	            uint32_t new_hz;

	            uint32_t decel_start = (z_total_steps > accel)
	                                    ? (z_total_steps - accel) : 0;

	            if (z_stepped < accel) {
	                // 가속 구간: 선형 보간
	                new_hz = g_z_start_hz
	                       + (g_z_max_hz - g_z_start_hz) * z_stepped / accel;
	            }
	            else if (z_stepped >= decel_start) {
	                // 감속 구간: 선형 보간
	                uint32_t steps_left = z_remain; // 남은 스텝
	                if (steps_left < accel) {
	                    new_hz = g_z_start_hz
	                           + (g_z_max_hz - g_z_start_hz) * steps_left / accel;
	                } else {
	                    new_hz = g_z_max_hz;
	                }
	            }
	            else {
	                // 정속 구간
	                new_hz = g_z_max_hz;
	            }

	            // 최저속 보장
	            if (new_hz < g_z_start_hz) new_hz = g_z_start_hz;

	            Z_StepPWM_SetHz(new_hz);

	            if (z_remain == 0) Z_Stop();
	        }
	    }
}

//	 거리 측정 후 X 이동
static void MeasureAndMoveX(void)
{
  // 거리센서 읽기
  uint16_t raw = Sensor_Distance_ReadRawAvg(16);
  float v  = (3.3f * raw) / 4095.0f;
  float cm = Sensor_Distance_VoltageToCm(v);

  // 기준값 대비 오차(mm)
  float err_mm = (cm - g_x_ref_cm) * 10.0f;

  // ±20mm(2cm) 클램핑 추가
  if (err_mm >  20.0f) err_mm =  20.0f;
  if (err_mm < -20.0f) err_mm = -20.0f;

  // 오차가 작으면 X축 이동 안 함, 후에 파종 시작
  if (fabsf(err_mm) <= g_deadband_mm) {
    state = ST_SOW_Z_DOWN;;
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
        	        state   = ST_SOW_Z_DOWN;
        	    }
            // 타이머 콜백에서 ST_DONE으로 변경
            break;

        case ST_SOW_Z_DOWN:
            Z_StartMoveSteps(-(int32_t)(g_sow_z_down_mm * g_z_steps_per_mm), g_z_move_hz);
            state = ST_SOW_WAIT_Z_DOWN;
            break;
        case ST_SOW_WAIT_Z_DOWN:
            if (!z_moving) state = ST_SOW_Z_UP;
            break;

        case ST_SOW_Z_UP:
            Z_StartMoveSteps(+(int32_t)(g_sow_z_down_mm * g_z_steps_per_mm), g_z_move_hz);
            state = ST_SOW_WAIT_Z_UP;
            break;

        case ST_SOW_WAIT_Z_UP:
            if (!z_moving) {
                sow_col++;
                if (sow_col < SOW_COLS) {
                    state = ST_SOW_NEXT_X;
                } else {
                    sow_col = 0;
                    sow_row++;
                    if (sow_row < SOW_ROWS)
                        state = ST_SOW_RETURN_X;
                    else
                        state = ST_EXIT_CONVEY;
                }
            }
            break;
        case ST_SOW_NEXT_X:
            X_StartMoveSteps(+(int32_t)(g_sow_x_pitch_mm * g_x_steps_per_mm), g_x_move_hz);
            state = ST_SOW_WAIT_X;
            break;
        case ST_SOW_WAIT_X:
            if (!x_moving) state = ST_SOW_Z_DOWN;
            break;
        case ST_SOW_RETURN_X:
            X_StartMoveSteps(-(int32_t)(g_sow_x_pitch_mm * (SOW_COLS-1) * g_x_steps_per_mm), g_x_move_hz);
            state = ST_SOW_WAIT_RETURN_X;
            break;
        case ST_SOW_WAIT_RETURN_X:
            if (!x_moving) state = ST_SOW_NEXT_ROW;
            break;
        case ST_SOW_NEXT_ROW:
            FirstConvey_MoveDistance(sow_row_feed_mm[sow_row - 1]);
            state = ST_SOW_WAIT_ROW;
            break;
        case ST_SOW_WAIT_ROW:
            if (FirstConvey_IsMoveDone()) state = ST_SOW_Z_DOWN;
            break;

        case ST_EXIT_CONVEY:
            if (!Sensor_IR_Detected()) {
                FirstConvey_ForceStop();
                state = ST_DONE;
            } else if (FirstConvey_IsMoveDone()) {
                FirstConvey_MoveDistance(10.0f);  // 10mm씩 계속 전진
            }
            break;
        case ST_DONE:
            // Z축 파종 추가 예정
            break;

        default:
            state = ST_RUN_CONVEY;
            break;
    }
}
bool Cartesian_IsCycleDone(void)
{
    return (state == ST_DONE);
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
