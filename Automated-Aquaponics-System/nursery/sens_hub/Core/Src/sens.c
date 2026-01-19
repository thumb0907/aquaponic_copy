#include "main.h"
#include "sens.h"


uint8_t tds_flag = 0; 		 // adc1 업데이트 플래그
uint8_t ph_flag = 0;  		// adc2 업데이트 플래그
uint32_t adc1_tds;			//adc1 값 tds
uint32_t adc2_ph;			// adc2값 ph
float ph_value;			//ph 캘리브레이션까지 적용
uint32_t tds_value;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
	  adc1_tds = HAL_ADC_GetValue(hadc);
	  tds_value = 2*adc1_tds;
  }
  if (hadc->Instance == ADC2)
  {
	  adc2_ph =HAL_ADC_GetValue(hadc);
	  ph_value = 7.0 + (adc2_ph - 482.5)*(4.0 - 7.0)/(635-482.5);
	  //7.0f + ((float)adc - ADC_PH7) * (4.0f - 7.0f) / (ADC_PH4 - ADC_PH7);
  }
}



