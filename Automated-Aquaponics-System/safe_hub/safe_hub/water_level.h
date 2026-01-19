#ifndef WATER_LEVEL_H
#define WATER_LEVEL_H

#include <Arduino.h>

extern const int wl_pin[2];
extern const uint8_t buz_pin;
extern int rawval1; 
extern int rawval2; 
extern float wl_mm1;
extern float wl_mm2;

void lev_pin();                              // 수위센서 핀 설정
void lev();                                  // 수위센서 구동함수

#endif