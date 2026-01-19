#include "dht.h"
#include <stdio.h>

// dht 센서 변수=============================
float tCelsius = 0; // 섭씨
float tFahrenheit = 0; //화씨
float RH = 0; // 습도
uint16_t dht_flag = 0;
//=========================================

static void dht_setoutput(void)
{
	GPIO_InitTypeDef gpio = {0};
	gpio.Pin = DHT22_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_OD;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DHT22_PORT, &gpio);
}

static void dht_setinput(void)
{
	GPIO_InitTypeDef gpio = {0};
	gpio.Pin = DHT22_PIN;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT22_PORT, &gpio);
}

uint8_t dht_start(void)
{
	uint8_t response = 0;

	dht_setoutput();
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_RESET);
	microDelay(1300);
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);
	microDelay(30);

	dht_setinput();
	microDelay(40);

	if (!HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))
	{
		microDelay(80);
		if (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)) response = 1;
	}

	return response;
}

uint8_t dht_read (void)
{
  uint8_t a,b = 0;
  for (a = 0; a < 8; a++)
  {
    while (!HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN));
    microDelay (40);
    if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
      b &= ~(1<<(7-a));								// 0
    else
      b |= (1<<(7-a));								// 1

    while (HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN));
  }
  return b;
}

uint8_t DHT22_Read(float *tempC, float *hum)
{
  uint8_t RH1, RH2, TC1, TC2, SUM;
  uint8_t check;

  if (!dht_start()) return 0;

  RH1 = dht_read();
  RH2 = dht_read();
  TC1 = dht_read();
  TC2 = dht_read();
  SUM = dht_read();

  check = (RH1 + RH2 + TC1 + TC2) & 0xFF;
  if (check != SUM) return 0;

  uint16_t rawH = (RH1 << 8) | RH2;
  uint16_t rawT = (TC1 << 8) | TC2;

  *hum = rawH / 10.0f;

  if (rawT & 0x8000) {
    rawT &= 0x7FFF;
    *tempC = -(rawT / 10.0f);
  } else {
    *tempC = rawT / 10.0f;
  }

  return 1;
}
