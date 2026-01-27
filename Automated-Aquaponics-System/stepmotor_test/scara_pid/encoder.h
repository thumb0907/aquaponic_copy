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
extern const long PULSES_PER_REV;
extern float J2_LEAD_MM_PER_REV;
extern unsigned long J2_DEFAULT_PPS;
extern volatile bool j2_endstop_hit;

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

void set_int();
///////////////////////////////////////////////////////////////////////////////////
void j1stepPulse();
void j1EncoderA();
bool move_j1(float targetAngle, float tolDeg = 1.0f);   
void move_j1_wait(float targetAngle,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 8000);
void enc_reset_j1();
void j1_home_stop_on_switch(bool dir_to_switch, unsigned long pps);
///////////////////////////////////////////////////////////////////////////////////
bool move_j2(float deg, unsigned long pps);
void j2EndstopISR();
///////////////////////////////////////////////////////////////////////////////////
void j3stepPulse();
void j3EncoderA();
bool move_j3(float targetAngle, float tolDeg = 1.0f);
void move_j3_wait(float targetAngle,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 8000);
void enc_reset_j3();
void j3_home_stop_on_switch(bool dir_to_switch, unsigned long pps);
///////////////////////////////////////////////////////////////////////////////////
void j4stepPulse();
void j4EncoderA();
void move_j4_wait(float targetAngle,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 8000);

bool move_j4(float targetAngle, float tolDeg = 1.0f);
void enc_reset_j4();
bool j4_home_stop_on_switch_safe(bool dir_to_switch, unsigned long pps);
///////////////////////////////////////////////////////////////////////////////////

void enc_reset_all();          
#endif
