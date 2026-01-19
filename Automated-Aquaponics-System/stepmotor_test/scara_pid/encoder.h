#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <TimerFour.h>
#include <TimerFive.h>
#include <math.h>

extern const float cpr;
extern const float en_cnt;
extern volatile bool j1_run, j2_run, j3_run, j4_run;

typedef struct {
  uint8_t pinA;
  uint8_t pinB;
  volatile long pos;
} encod;

extern encod j1_enc;
//extern encod j2_enc;
extern encod j3_enc;
extern encod j4_enc;

extern volatile bool j1pulseState;
//extern volatile bool j2pulseState;
extern volatile bool j3pulseState;
extern volatile bool j4pulseState;

extern volatile long pulseInterval;

float encoder_getAngleDeg(const encod* e);

void set_tim();
void set_int();

void j1stepPulse();
//void j2stepPulse();
void j3stepPulse();
void j4stepPulse();

void j1EncoderA();
//void j2EncoderA();
void j3EncoderA();
void j4EncoderA();
void set_int();

void move_j1(float targetAngle);   
//void move_j2(float targetAngle);
void move_j2(float deg, unsigned long pps);
void move_j3(float targetAngle);
void move_j4(float targetAngle);
#endif
