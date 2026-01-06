#include "encoder.h"
#include "set_motor.h"
#include "pid.h"
#include <math.h>

extern float targetAngle;  

const float cpr    = 400.0f;
const float en_cnt = cpr * 2.0f;

encod j1_enc = { j1_A, j1_B, 0 };
encod j2_enc = { j2_A, j2_B, 0 };
encod j3_enc = { j3_A, j3_B, 0 };
encod j4_enc = { j4_A, j4_B, 0 };

volatile bool pulseState = false;
volatile long pulseInterval = 100;

float encoder_getAngleDeg(const encod* e) {
  return -(e->pos) * 360.0f / en_cnt;
}

void set_int()
{
  attachInterrupt(digitalPinToInterrupt(j1_enc.pinA), j1EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j2_enc.pinA), j2EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j3_enc.pinA), j3EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j4_enc.pinA), j4EncoderA, CHANGE);
}

void stepPulse() {
  digitalWrite(j1_pul, pulseState);
  pulseState = !pulseState;
}

void j1EncoderA() {
  bool a = digitalRead(j1_enc.pinA);
  bool b = digitalRead(j1_enc.pinB);
  j1_enc.pos += (a == b) ? 1 : -1;
}
void j2EncoderA() {
  bool a = digitalRead(j2_enc.pinA);
  bool b = digitalRead(j2_enc.pinB);
  j1_enc.pos += (a == b) ? 1 : -1;
}
void j3EncoderA() {
  bool a = digitalRead(j3_enc.pinA);
  bool b = digitalRead(j3_enc.pinB);
  j1_enc.pos += (a == b) ? 1 : -1;
}
void j4EncoderA() {
  bool a = digitalRead(j4_enc.pinA);
  bool b = digitalRead(j4_enc.pinB);
  j1_enc.pos += (a == b) ? 1 : -1;
}

void move_j1()
{
  noInterrupts();
  long Count = j1_enc.pos;
  interrupts();

  encod snap = j1_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = targetAngle - nowAngle;
  float pidOut = pid_update(&j1_pid, error);

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 5000) speed = 5000;

  long interval = 1000000L / speed;

  noInterrupts();
  pulseInterval = interval;
  Timer1.setPeriod(pulseInterval);
  interrupts();

  digitalWrite(j1_dir, (pidOut > 0) ? LOW : HIGH);

  Serial.print("Angle="); Serial.print(nowAngle);
  Serial.print(" Error="); Serial.print(error);
  Serial.print(" speed="); Serial.print(speed);
  Serial.print(" interval(us)="); Serial.println(interval);
}

void move_j2()
{
  noInterrupts();
  long Count = j2_enc.pos;
  interrupts();

  encod snap = j2_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = targetAngle - nowAngle;
  float pidOut = pid_update(&j2_pid, error);

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 5000) speed = 5000;

  long interval = 1000000L / speed;

  noInterrupts();
  pulseInterval = interval;
  Timer3.setPeriod(pulseInterval);
  interrupts();

  digitalWrite(j2_dir, (pidOut > 0) ? LOW : HIGH);

  Serial.print("Angle="); Serial.print(nowAngle);
  Serial.print(" Error="); Serial.print(error);
  Serial.print(" speed="); Serial.print(speed);
  Serial.print(" interval(us)="); Serial.println(interval);
}
