// cartesian.h
#ifndef CARTESIAN_H
#define CARTESIAN_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

void Cartesian_Task(void);

void Cartesian_SetXStepsPerMM(float steps_per_mm);
void Cartesian_SetXRefCm(float ref_cm);

void Cartesian_OnTimPeriodElapsed(TIM_HandleTypeDef *htim);
//void Cartesian_OnTimOcCallback(TIM_HandleTypeDef *htim);
bool Cartesian_IsBusy(void);

void Cartesian_HomingTask(void);
void Cartesian_StartHoming(void);
bool Cartesian_IsHomingDone(void);
void Cartesian_limit(uint16_t GPIO_Pin);

void Cartesian_ResetSequence(void);
bool Cartesian_IsCycleDone(void);
bool Cartesian_IsHomingStarted(void);
// 테스트할때 사용
void Cartesian_TestMoveX(int32_t steps, uint32_t hz);
void Cartesian_TestMoveZ(int32_t steps, uint32_t hz);
void Cartesian_TestStopX(void);
void Cartesian_TestStopZ(void);

#endif
