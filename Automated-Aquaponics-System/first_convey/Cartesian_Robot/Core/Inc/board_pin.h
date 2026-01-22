#ifndef BOARD_PIN_H
#define BOARD_PIN_H

#include "stm32f4xx_hal.h"

// 직교로봇 스텝모터
#define CAR_X_DIR_PIN
#define CAR_X_DIR_PORT
#define CAR_X_EN_PIN
#define CAR_X_EN_PORT

#define CAR_Z_DIR_PIN
#define CAR_Z_DIR_PORT
#define CAR_Z_EN_PIN
#define CAR_Z_EN_PORT

// 컨베이어 스텝모터
#define CON_DIR_PORT GPIOC
#define CON_DIR_PIN  GPIO_PIN_9

#define CON_EN_PORT  GPIOC
#define CON_EN_PIN   GPIO_PIN_8

// 적외선 센서
#define IR_PORT  GPIOC
#define IR_PIN   GPIO_PIN_3

#endif
