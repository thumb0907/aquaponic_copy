#ifndef PUMP_MOTOR_H
#define PUMP_MOTOR_H

#define off true                              //펌프모터 on off 메크로
#define on false

#include <Arduino.h>

extern const uint8_t rel_pin[4];


void pump_pin();                              //펌프모터 릴레이 핀 설정
void pump(bool a, bool b, bool c, bool d);    // 펌프모터 구동함수

#endif
