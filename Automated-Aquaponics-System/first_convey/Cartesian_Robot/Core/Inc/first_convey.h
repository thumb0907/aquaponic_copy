// first_convey.h
#ifndef FIRST_CONVEY_H
#define FIRST_CONVEY_H

#include <stdbool.h>
void FirstConvey_Reset(void);
void FirstConvey_Task(void);
bool FirstConvey_IsStopped(void);

void FirstConvey_SetIrEnabled(bool en);
void FirstConvey_MoveDistance(float mm);
bool FirstConvey_IsMoveDone(void);
void FirstConvey_ForceStop(void);
void FirstConvey_OnTimPeriodElapsed(TIM_HandleTypeDef*);

void FirstConvey_SendPulses(uint32_t pulses); // 컨베이어 파라미터 찾는용도
#endif
