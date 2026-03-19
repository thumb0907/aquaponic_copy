// sequence.h
#ifndef SEQUENCE_H
#define SEQUENCE_H

void Sequence_Init(void);       // main.c USER CODE BEGIN 2 에서 호출
void App_Run(void);             // while(1) 안에서 매 루프 호출
void Sequence_OnLimitHit(void); // HAL_GPIO_EXTI_Callback 에서 호출

#endif
