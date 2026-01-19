#include "main.h"
#include "sens.h"

uint32_t adc1_value;
uint8_t adc1_flag = 0;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc1_value = HAL_ADC_GetValue(hadc);
    adc1_flag = 1;
  }
}
