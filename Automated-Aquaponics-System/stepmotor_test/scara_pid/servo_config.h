#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH     600
#define MAX_PULSE_WIDTH     2800
#define FREQUENCY           50
#define SERVO_CHANNEL       0

void setServo();
void grip(bool x);

void grip_timer_on();
void grip_timer_off();

void smove(uint8_t degree);

#endif