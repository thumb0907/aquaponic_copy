#ifndef EMERGENCY_H
#define EMERGENCY_H

#include <Arduino.h>

extern const byte stop_pin;
extern const byte start_pin;
extern const byte sw_pin;
extern const byte led_pin;
extern bool stop;
extern bool start;
extern bool sw;

void emergency_pin();
void stopISR();
void startISR();
bool SW();

# endif