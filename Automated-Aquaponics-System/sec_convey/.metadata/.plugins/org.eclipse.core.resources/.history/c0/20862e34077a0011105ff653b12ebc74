#include "sec_convey.h"

extern TIM_HandleTypeDef htim3;

static uint32_t TIM_APB1_GetTimerClockHz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2;
    return pclk1;
}

static void TIM3_SetStepHz(uint32_t hz)
{
    if (hz < 1) hz = 1;

    uint32_t tim_clk = TIM_APB1_GetTimerClockHz();
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

void Conveyor_Enable(bool en)
{
    // MKS SERVO57D: EN LOW = 활성화
    HAL_GPIO_WritePin(CON_PORT, CON_EN_PIN,
                      en ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Conveyor_SetDir(bool dir)
{
    HAL_GPIO_WritePin(CON_PORT, CON_DIR_PIN,
                      dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
