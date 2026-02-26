	// sensor.c
	#include "stm32f4xx_hal.h"
	#include "sensor.h"
	#include "board_pin.h"
	#include "main.h"
	#include <math.h>

	extern ADC_HandleTypeDef hadc1;

	//////////////// 적외선센서
	bool Sensor_IR_Detected(void)
	{
	  return (HAL_GPIO_ReadPin(IR_PORT, IR_PIN) == GPIO_PIN_RESET); // LOW면 감지
	}

	//////////////////////////  거리감지센서
	uint16_t Sensor_Distance_ReadRawOnce(void)
	{
	  HAL_ADC_Start(&hadc1);
	  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		return v;
	  }
	  HAL_ADC_Stop(&hadc1);
	  return 0;
	}
	uint16_t Sensor_Distance_ReadRawAvg(uint8_t n)
	{
	  if (n < 1) n = 1;

	  uint32_t sum = 0;
	  for (uint8_t i = 0; i < n; i++)
	  {
		sum += Sensor_Distance_ReadRawOnce();
		HAL_Delay(2);
	  }
	  return (uint16_t)(sum / n);
	}

	float Sensor_Distance_RawToVoltage(uint16_t raw)
	{
	  return (3.3f * (float)raw) / 4095.0f;
	}
	/*
	float Sensor_Distance_VoltageToCm(float v)
	{
	  if (v < 0.2f) return 999.0f; // 측정 불가
	  float cm = 27.86f * powf(v, -1.15f);

	  // 센서 유효 범위: 2cm ~ 15cm
	  if (cm < 2.0f)  cm = 2.0f;
	  if (cm > 15.0f) cm = 15.0f;

	  return cm;
	}*/

	// sensor.c의 Sensor_Distance_VoltageToCm 함수를 이렇게 수정: 선형 보간
	float Sensor_Distance_VoltageToCm(float v)
	{
	  if (v < 0.3f) return 15.0f;

	  // 실측 데이터: {전압, 거리}
	  typedef struct {
		float voltage;
		float distance;
	  } CalibPoint;

	  static const CalibPoint table[] = {
		{1.89f,  2.0f},
		{1.55f,  3.0f},
		{1.24f,  4.0f},
		{1.05f,  5.0f},
		{0.86f,  6.0f},
		{0.74f,  7.0f},
		{0.71f,  8.0f},
		{0.63f,  9.0f},
		{0.56f, 10.0f},
		{0.47f, 11.0f},
		{0.43f, 12.0f},
		{0.37f, 13.0f},
		{0.33f, 14.0f}
	  };

	  const uint8_t table_size = sizeof(table) / sizeof(table[0]);

	  // 범위 밖 처리
	  if (v >= table[0].voltage) return table[0].distance;  // 2cm 이하
	  if (v <= table[table_size-1].voltage) return 15.0f;   // 15cm 이상

	  // 선형 보간
	  for (uint8_t i = 0; i < table_size - 1; i++)
	  {
		if (v <= table[i].voltage && v >= table[i+1].voltage)
		{
		  float v1 = table[i].voltage;
		  float v2 = table[i+1].voltage;
		  float d1 = table[i].distance;
		  float d2 = table[i+1].distance;

		  // 선형 보간 공식: d = d1 + (v - v1) * (d2 - d1) / (v2 - v1)
		  float distance = d1 + (v - v1) * (d2 - d1) / (v2 - v1);
		  return distance;
		}
	  }

	  return 15.0f;
	}
