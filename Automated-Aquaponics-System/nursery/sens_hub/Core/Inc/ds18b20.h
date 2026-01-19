#ifndef DS18B20_H
#define DS18B20_H

#include "main.h"
#include <stdint.h>

#define DS18B20_PORT GPIOB
#define DS18B20_PIN  GPIO_PIN_0

// 너가 이미 쓰는 microDelay(us) 함수 (DWT든 TIM이든)
void microDelay(uint16_t us);

uint8_t DS18B20_StartConversion(void);     // Convert T 시작
uint8_t DS18B20_ReadTemp(float *tempC);    // 변환된 온도 읽기

#endif
