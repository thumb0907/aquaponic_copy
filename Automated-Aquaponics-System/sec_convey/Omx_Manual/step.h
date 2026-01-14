#pragma once
#include <Arduino.h>

extern const int PUL;
extern const int DIR;
extern bool dir_state;

void stepPulse(bool dir, int speed);