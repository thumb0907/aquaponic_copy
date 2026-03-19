// sensor.c 센서 관련
#include "sensor.h"

// 적외선센서
uint8_t IR_Detected(void)
{
    return (HAL_GPIO_ReadPin(IR_PORT, IR_PIN) == GPIO_PIN_RESET);
}

// 리밋스위치 (NO: 눌리면 LOW, RESET)
uint8_t Z_LimitHit(void)
{
    return (HAL_GPIO_ReadPin(Z_LIM_PORT, Z_LIM_PIN) == GPIO_PIN_RESET);
}
