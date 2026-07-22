// sensor.h
#ifndef SENSOR_H
#define SENSOR_H
#include "stm32f4xx_hal.h"
#include "pin.h"

uint8_t IR_Detected(void); // 적외선센서
uint8_t Z_PhotoDetected(void);  // 리밋스위치

#endif
