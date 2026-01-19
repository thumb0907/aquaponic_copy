#ifndef SENS_H
#define SENS_H

#include <stdint.h>

extern uint8_t tds_flag; 		 // adc1 업데이트 플래그
extern uint8_t ph_flag;  		// adc2 업데이트 플래그
extern uint32_t tds_value; 		//adc1 버퍼
extern float ph_value; 		//adc2 버퍼

#endif
