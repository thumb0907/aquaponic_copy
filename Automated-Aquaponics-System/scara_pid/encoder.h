#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <TimerOne.h>
#include <TimerThree.h>

extern const float cpr;
extern const float en_cnt;

typedef struct {
  uint8_t pinA;
  uint8_t pinB;
  volatile long pos;
} encod;

extern encod j1_enc;
extern encod j2_enc;
extern encod j3_enc;
extern encod j4_enc;

extern volatile bool pulseState;
extern volatile long pulseInterval;

float encoder_getAngleDeg(const encod* e);

void stepPulse();
void j1EncoderA();
void j2EncoderA();
void j3EncoderA();
void j4EncoderA();
void set_int();
void move_j1();   

#endif
