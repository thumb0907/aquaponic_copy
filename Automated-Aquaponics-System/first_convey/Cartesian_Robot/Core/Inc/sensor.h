#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

bool     Sensor_IR_Detected(void);
uint16_t Sensor_DMS80_ReadRawAvg(int n);
float    Sensor_DMS80_VoltageToCm(float v);
float    Sensor_DMS80_ReadCmAvg(int n);

#endif
