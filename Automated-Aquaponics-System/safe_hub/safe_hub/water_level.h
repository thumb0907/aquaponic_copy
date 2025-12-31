#ifndef WATER_LEVEL_H
#define WATER_LEVEL_H

#include <Arduino.h>

#define wl A0                                // 수위센서 핀 번호

void lev_pin();                              // 수위센서 핀 설정
void lev();                                  // 수위센서 구동함수

#endif