// board_pin.h
#ifndef BOARD_PIN_H
#define BOARD_PIN_H

#include "stm32f4xx_hal.h"

/////// TIM2- 직교로봇X축(PA5), TIM3- 컨베이어(PA6),TIM5- 직교로봇Z축(PA0)

// 직교로봇 스텝모터
#define CAR_X_DIR_PIN   GPIO_PIN_1
#define CAR_X_DIR_PORT	GPIOB
#define CAR_X_EN_PIN	GPIO_PIN_2
#define CAR_X_EN_PORT	GPIOB

#define CAR_Z_DIR_PIN	GPIO_PIN_13
#define CAR_Z_DIR_PORT	GPIOB
#define CAR_Z_EN_PIN	GPIO_PIN_14
#define CAR_Z_EN_PORT	GPIOB

// 직교로봇 리밋스위치(EXTI)
#define X_LIM_PORT		GPIOB
#define X_LIM_PIN		GPIO_PIN_3

#define Z_LIM_PORT		GPIOB
#define Z_LIM_PIN		GPIO_PIN_5

// 컨베이어 스텝모터
#define CON_DIR_PORT GPIOC
#define CON_DIR_PIN  GPIO_PIN_9

#define CON_EN_PORT  GPIOC
#define CON_EN_PIN   GPIO_PIN_8

// 적외선 센서
#define IR_PORT  GPIOC
#define IR_PIN   GPIO_PIN_3

// 파종 펌프모터
#define PUMP_PORT	GPIOC
#define PUMP_PIN	GPIO_PIN_0

#endif
