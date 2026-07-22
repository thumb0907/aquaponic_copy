// sensor.c 센서 관련
#include "sensor.h"

// 적외선센서
uint8_t IR_Detected(void)
{
    return (HAL_GPIO_ReadPin(IR_PORT, IR_PIN) == GPIO_PIN_RESET);
}

uint8_t Z_PhotoDetected(void)
{
    return (
        HAL_GPIO_ReadPin(
            Z_PHOTO_PORT,
            Z_PHOTO_PIN
        ) == GPIO_PIN_SET
    );
}
