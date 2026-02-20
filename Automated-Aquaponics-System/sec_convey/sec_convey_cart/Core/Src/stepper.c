#include "stepper.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim3; // TIM3 CH1 컨베이어
extern TIM_HandleTypeDef htim5; // TIM5 CH1 직교로봇

static uint32_t TIM_APB1_GetTimerClockHz(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2;
  return pclk1;
}

//////////////////////////////////////////// 컨베이어
void Conveyor_Enable(bool en)
{
  HAL_GPIO_WritePin(CON_PORT, CON_EN_PIN, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Conveyor_SetDir(bool dir)
{
  HAL_GPIO_WritePin(CON_PORT, CON_DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void TIM3_SetStepHz(uint32_t hz)
{
  if (hz < 1) hz = 1;

  uint32_t tim_clk = TIM_APB1_GetTimerClockHz();

  // tick을 1MHz로 만들기(분해능 좋게)
  uint32_t psc = (tim_clk / 1000000UL);
  if (psc < 1) psc = 1;
  psc -= 1;

  uint32_t tick_hz = tim_clk / (psc + 1);
  uint32_t arr = (tick_hz / hz) - 1;
  if (arr > 0xFFFF) arr = 0xFFFF;

  __HAL_TIM_SET_PRESCALER(&htim3, psc);
  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (arr + 1) / 2);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
}

void Conveyor_StartHz(uint32_t step_hz)
{
  TIM3_SetStepHz(step_hz);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void Conveyor_Stop(void)
{
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

void Conveyor_RampToHz(uint32_t target_hz, uint32_t step_hz, uint32_t step_ms)
{
  static uint32_t cur_hz = 0;

  if (step_hz < 1) step_hz = 1;
  if (step_ms < 1) step_ms = 1;

  // 첫 호출에서 PWM이 아직 안 켜져 있으면, 아주 낮은 속도로 시작
  if (cur_hz == 0 && target_hz > 0) {
    cur_hz = (target_hz < 200) ? target_hz : 200; // 시작값(너무 낮으면 떨릴 수 있어 200 권장)
    TIM3_SetStepHz(cur_hz);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  }

  // 목표가 0이면: 감속 후 정지
  if (target_hz == 0) {
    while (cur_hz > 0) {
      if (cur_hz <= step_hz) cur_hz = 0;
      else cur_hz -= step_hz;

      if (cur_hz == 0) break;

      TIM3_SetStepHz(cur_hz);
      HAL_Delay(step_ms);
    }
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    cur_hz = 0;
    return;
  }

  // 가속/감속 램프
  while (cur_hz != target_hz) {
    if (cur_hz < target_hz) {
      uint32_t next = cur_hz + step_hz;
      cur_hz = (next > target_hz) ? target_hz : next;
    } else {
      // cur_hz > target_hz
      uint32_t next = (cur_hz <= step_hz) ? 0 : (cur_hz - step_hz);
      cur_hz = (next < target_hz) ? target_hz : next;
    }

    TIM3_SetStepHz(cur_hz);
    HAL_Delay(step_ms);
  }
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

void Z_MoveSteps(int32_t steps, uint32_t step_hz)
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
