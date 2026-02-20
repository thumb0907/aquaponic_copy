// sensor.h
#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

// 적외선센서
bool     Sensor_IR_Detected(void);

// 거리센서
uint16_t Sensor_Distance_ReadRawOnce(void);
uint16_t Sensor_Distance_ReadRawAvg(uint8_t n);
float    Sensor_Distance_RawToVoltage(uint16_t raw);
float    Sensor_Distance_VoltageToCm(float v);

// 직교로봇 리밋스위치

#endif
