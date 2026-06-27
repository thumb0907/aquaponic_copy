	// sensor.c
	#include "stm32f4xx_hal.h"
	#include "sensor.h"
	#include "board_pin.h"
	#include "main.h"

	//////////////// 적외선센서
	bool Sensor_IR_Detected(void)
	{
	  return (HAL_GPIO_ReadPin(IR_PORT, IR_PIN) == GPIO_PIN_RESET); // LOW면 감지
	}
