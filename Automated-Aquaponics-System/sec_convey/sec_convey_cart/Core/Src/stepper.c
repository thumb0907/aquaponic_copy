// stepper.c 직교로봇
#include "stepper.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim5; // TIM5 CH1 직교로봇 z축만 구동

static uint32_t TIM_APB1_GetTimerClockHz(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2;
  return pclk1;
}

/////////////////////////////////////////////// 직교로봇
static volatile uint32_t z_remain = 0;
static volatile bool z_busy = false;

void Z_Enable(bool en)
{
  HAL_GPIO_WritePin(CART_PORT, CART_EN_PIN, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Z_SetDir(bool dir)
{
  HAL_GPIO_WritePin(CART_PORT, CART_DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void TIM5_SetStepHz(uint32_t hz)
{
  if (hz < 1) hz = 1;

  uint32_t tim_clk = TIM_APB1_GetTimerClockHz();

  uint32_t psc = (tim_clk / 1000000UL);
  if (psc < 1) psc = 1;
  psc -= 1;

  uint32_t tick_hz = tim_clk / (psc + 1);
  uint32_t arr = (tick_hz / hz) - 1;  // TIM5는 32bit라 상관 없음

  __HAL_TIM_SET_PRESCALER(&htim5, psc);
  __HAL_TIM_SET_AUTORELOAD(&htim5, arr);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (arr + 1) / 2);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
  HAL_TIM_GenerateEvent(&htim5, TIM_EVENTSOURCE_UPDATE);
}

void Z_Stop(void)
{
  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
  __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);

  Z_Enable(false);

  z_remain = 0;
  z_busy = false;
}

bool Z_IsBusy(void)
{
  return z_busy;
}

void Z_MoveSteps(int32_t steps, uint32_t step_hz) // step은 pulse, step_hz는 초당 펄스 수, 8000, 4000이라면 8000/4000 = 2초
{
  if (steps == 0) return;

  bool dir = (steps > 0);
  uint32_t n = (uint32_t)abs(steps);

  Z_SetDir(dir);
  Z_Enable(true);

  z_remain = n;
  z_busy = true;

  TIM5_SetStepHz(step_hz);

  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
}

// main.c 콜백에서 호출
void Z_OnTimUpdate(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM5) return;
  if (!z_busy) return;

  if (z_remain > 0)
  {
    z_remain--;
    if (z_remain == 0)
    {
      Z_Stop();
    }
  }
}
