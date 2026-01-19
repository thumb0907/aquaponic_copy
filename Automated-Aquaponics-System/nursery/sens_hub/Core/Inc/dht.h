#ifndef DHT_H
#define DHT_H

#include <stdint.h>
#include <stdio.h>
#include "main.h"

#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_4

// dht 변수==================================
extern uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;
extern uint32_t pMillis, cMillis;
extern float tCelsius; 							// 섭씨
extern float tFahrenheit;						//화씨
extern float RH; 								// 습도
extern uint16_t dht_flag;

uint8_t dht_start(void); 						// dht 시작
uint8_t dht_read (void); 						// dht 1바이트 읽기
uint8_t DHT22_Read(float *tempC, float *hum); 	// dht 전체 데이터 읽기
//=========================================

#endif
