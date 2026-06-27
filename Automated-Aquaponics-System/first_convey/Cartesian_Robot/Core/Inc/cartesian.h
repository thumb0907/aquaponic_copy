#ifndef CARTESIAN_H
#define CARTESIAN_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum {
    CART_STAGE_UNKNOWN = 0,
    CART_STAGE_RUN_CONVEY,
	CART_STAGE_PICKING,
    CART_STAGE_SEEDING,
    CART_STAGE_EJECTING,
    CART_STAGE_DONE
} CartesianStage;

void Cartesian_Task(void);

void Cartesian_SetXStepsPerMM(float steps_per_mm);

void Cartesian_OnTimPeriodElapsed(TIM_HandleTypeDef *htim);

void Cartesian_HomingTask(void);
void Cartesian_StartHoming(void);
bool Cartesian_IsHomingDone(void);
bool Cartesian_IsHomingStarted(void);
bool Cartesian_IsHomingError(void);   // 추가
void Cartesian_limit(uint16_t GPIO_Pin);

void Cartesian_ResetSequence(void);
bool Cartesian_IsCycleDone(void);

CartesianStage Cartesian_GetStage(void);

/* 테스트용 */
void Cartesian_TestMoveX(int32_t steps, uint32_t hz);
void Cartesian_TestMoveZ(int32_t steps, uint32_t hz);
void Cartesian_TestStopX(void);
void Cartesian_TestStopZ(void);
void Cartesian_TestPump(void);          // 펌프 5초 ON 후 OFF
void Cartesian_TestPickCycle(void);     // 픽업 한 사이클
void Cartesian_TestPickSowCycle(void);  // 픽업+파종 한 사이클
#endif
