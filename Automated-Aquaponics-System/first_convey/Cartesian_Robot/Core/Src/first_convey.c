#include "stm32f4xx_hal.h"
#include "first_convey.h"
#include "board_pin.h"
#include "sensor.h"
#include "main.h"
#include <stdio.h>

extern TIM_HandleTypeDef  htim3; // htim3, huart2는 main.c에 실제로 생성(정의) 되어 있고, 여기서는 “그걸 가져다 쓰겠다”는 선언만 한 것(extern).
extern UART_HandleTypeDef huart2;

typedef enum { CONVEYOR_RUN = 0, CONVEYOR_STOP } ConveyorState;
static ConveyorState state = CONVEYOR_RUN; 	// 현재 상태: 컨베이어 구동중
static bool motor_running = false; 			// 이미 모터를 켰는지 기억하는 플래그

// TIM3는 APB1 타이머. prescaler가 1이 아니면 TIM clk = PCLK1*2
static uint32_t TIM3_GetClockHz(void) 			// 타이머 클럭 계산
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2;
  return pclk1;
}
static void StepPWM_SetHz(uint32_t hz)      	// PWM 주파수 설정
{
  if (hz < 1) hz = 1;

  uint32_t tim_clk = TIM3_GetClockHz();
  uint32_t arr = (tim_clk / hz) - 1;
  if (arr > 0xFFFF) arr = 0xFFFF;

  __HAL_TIM_SET_PRESCALER(&htim3, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr / 2); // 50%
  __HAL_TIM_SET_COUNTER(&htim3, 0);
}

static void Motor_SetDir(bool dir) // dir
{
  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void Motor_Enable(bool en) // en
{
  HAL_GPIO_WritePin(EN_PORT, EN_PIN, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void Motor_Start(uint32_t step_hz) // motor start
{
  StepPWM_SetHz(step_hz);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}
static void Motor_Stop(void) // motor stop
{
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

static void Conveyor_Run(void) // 컨베이어 구동
{
  if (!motor_running)
  {
    Motor_SetDir(true);
    Motor_Enable(true);
    Motor_Start(2000);
    motor_running = true;
  }

  if (Sensor_IR_Detected())
  {
    Motor_Stop();
    Motor_Enable(false);
    motor_running = false;
    state = CONVEYOR_STOP;
  }
}

static void Conveyor_Stop(void) // 컨베이어 정지 -> 트레이 절대거리값 측정 후 출력
{
  uint16_t raw = Sensor_DMS80_ReadRawAvg(16);
  uint32_t mv = (3300UL * raw) / 4095UL;

  float v  = (3.3f * raw) / 4095.0f;
  float cm = Sensor_DMS80_VoltageToCm(v);

  char buf[80];
  int cm10 = (int)(cm * 10.0f);
  int len = snprintf(buf, sizeof(buf),
                     "raw=%u, %lumV, cm=%d.%d\r\n",
                     raw, mv, cm10/10, cm10%10);

  HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
  HAL_Delay(500);
}

void FirstConvey_Task(void) // 컨베이어 구동/정지 함수 (main에서 호출)
{
  if (state == CONVEYOR_RUN)  Conveyor_Run();
  else                        Conveyor_Stop();
}
