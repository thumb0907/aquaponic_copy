/* stepper.c  직교로봇 Z축 스텝 드라이버 */
#include "stepper.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim5;   /* TIM5 CH1 — Z축 PWM STEP 출력 */

/* ── APB1 타이머 클럭 계산 ─────────────────────── */
static uint32_t TIM_APB1_GetTimerClockHz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    /* APB1 분주가 1이 아니면 × 2 */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2;
    return pclk1;
}

/* ── 내부 상태 ───────────────────────────────── */
static volatile uint32_t z_remain = 0;
static volatile bool     z_busy   = false;

/* ── GPIO 제어 ───────────────────────────────── */
void Z_Enable(bool en)
{
    GPIO_PinState state =
        en ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(CART1_PORT, CART1_EN_PIN, state);
    HAL_GPIO_WritePin(CART2_PORT, CART2_EN_PIN, state);
}

void Z_SetDir(bool dir)
{
    GPIO_PinState state =
        dir ? GPIO_PIN_RESET : GPIO_PIN_SET;

    HAL_GPIO_WritePin(CART1_PORT, CART1_DIR_PIN, state);
    HAL_GPIO_WritePin(CART2_PORT, CART2_DIR_PIN, state);
}

/* ── TIM5 주파수 설정 ────────────────────────── */
static void TIM5_SetStepHz(uint32_t hz)
{
    if (hz < 1) hz = 1;

    uint32_t tim_clk = TIM_APB1_GetTimerClockHz();

    /* Prescaler: 타이머를 1 MHz 기준으로 고정 */
    uint32_t psc = (tim_clk / 1000000UL);
    if (psc < 1) psc = 1;
    psc -= 1;

    uint32_t tick_hz = tim_clk / (psc + 1);
    uint32_t arr     = (tick_hz / hz) - 1;  /* TIM5는 32bit — 오버플로 없음 */
    uint32_t pulse = (arr + 1) / 2;

    __HAL_TIM_SET_PRESCALER(&htim5, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim5, arr);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse);  /* 50 % 듀티 */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pulse);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    HAL_TIM_GenerateEvent(&htim5, TIM_EVENTSOURCE_UPDATE);
}

/* ── 이동 ────────────────────────────────────── */
void Z_MoveSteps(int32_t steps, uint32_t step_hz)
{
    if (steps == 0) return;

    bool     dir = (steps > 0);
    uint32_t n   = (uint32_t)abs(steps);

    Z_SetDir(dir);
    Z_Enable(true);
    /* Z_Enable(true)는 호출부에서 먼저 해야 함
     * (Z_Stop → Enable(false) 후 바로 MoveSteps 하면 드라이버가
     *  아직 비활성화 상태일 수 있어 호밍 완료 직후 10 ms 대기 필요) */

    z_remain = n;
    z_busy   = true;

    TIM5_SetStepHz(step_hz);

    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
}

/* ── 즉시 정지 ───────────────────────────────── */
static void Z_StopPulse(void)
{
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);

    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);

    z_remain = 0;
    z_busy   = false;
}

// 이동은 정지하지만 모터 드라이버는 활성 상태로 유지
void Z_StopHold(void)
{
    Z_StopPulse();
    Z_Enable(true);
}

// 이동 정지 후 모터 드라이버까지 비활성화
void Z_Stop(void)
{
    Z_StopPulse();
    Z_Enable(false);
}

/* ── 상태 조회 ───────────────────────────────── */
bool Z_IsBusy(void)
{
    return z_busy;
}

/* ── main.c HAL_TIM_PeriodElapsedCallback 에서 호출 ── */
void Z_OnTimUpdate(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM5) return;
    if (!z_busy) return;

    if (z_remain > 0) {
        z_remain--;
        if (z_remain == 0) {
            Z_Stop();
        }
    }
}
