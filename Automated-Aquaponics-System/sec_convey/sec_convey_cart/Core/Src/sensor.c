// sensor.c
#include "sensor.h"

uint8_t IR_Detected(void)
{
    return (HAL_GPIO_ReadPin(IR_PORT, IR_PIN) == GPIO_PIN_RESET);
}
