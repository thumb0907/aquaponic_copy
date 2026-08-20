// pin.h
#ifndef PIN_H
#define PIN_H

#include "stm32f4xx_hal.h"

// 컨베이어 pul- pa6 (tim3_ch1)
// 두번째 컨베이어
#define CON_DIR_PIN	  GPIO_PIN_9
#define CON_EN_PIN	  GPIO_PIN_8
#define CON_PORT	  GPIOC

// 직교로봇
// 직교로봇 모터 1
#define CART1_DIR_PIN    GPIO_PIN_12
#define CART1_EN_PIN     GPIO_PIN_11
#define CART1_PORT       GPIOA

// 직교로봇 모터 2
#define CART2_DIR_PIN    GPIO_PIN_10
#define CART2_EN_PIN     GPIO_PIN_11
#define CART2_PORT       GPIOC

// 적외선센서
#define IR_PIN		GPIO_PIN_3
#define IR_PORT		GPIOC

// 직교로봇 리밋스위치 포토
#define Z_PHOTO_PIN		GPIO_PIN_2
#define Z_PHOTO_PORT	GPIOC

#endif
