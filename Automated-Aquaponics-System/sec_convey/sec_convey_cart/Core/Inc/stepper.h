// stepper.h
#ifndef STEPPER_H
#define STEPPER_H

#include "stm32f4xx_hal.h"
#include "pin.h"
#include <stdbool.h>

// Z축
void Z_Enable(bool en);
void Z_SetDir(bool dir);
void Z_MoveSteps(int32_t steps, uint32_t step_hz); // +steps / -steps
bool Z_IsBusy(void);
void Z_Stop(void);

// main.c의 HAL_TIM_PeriodElapsedCallback에서 호출할 것
void Z_OnTimUpdate(TIM_HandleTypeDef *htim);

#endif
