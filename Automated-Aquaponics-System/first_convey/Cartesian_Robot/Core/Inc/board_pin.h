#ifndef BOARD_PIN_H
#define BOARD_PIN_H

#include "stm32f4xx_hal.h"

// 컨베이어 스텝모터
#define DIR_PORT GPIOC
#define DIR_PIN  GPIO_PIN_9

#define EN_PORT  GPIOC
#define EN_PIN   GPIO_PIN_8

// 적외선 센서
#define IR_PORT  GPIOC
#define IR_PIN   GPIO_PIN_3

#endif
