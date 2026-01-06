#include "pid.h"

PID   j1_pid = { 30.0f, 0.0f, 20.0f, 0.0f, 0.0f, 5000.0f };
PID   j2_pid = { 30.0f, 0.0f, 20.0f, 0.0f, 0.0f, 5000.0f };
PID   j3_pid = { 30.0f, 0.0f, 20.0f, 0.0f, 0.0f, 5000.0f };
PID   j4_pid = { 30.0f, 0.0f, 20.0f, 0.0f, 0.0f, 5000.0f };

float pid_update(PID *p, float error) {
  p->integral += error;
  if (p->integral >  p->iLimit) p->integral =  p->iLimit;
  if (p->integral < -p->iLimit) p->integral = -p->iLimit;

  float derivative = error - p->prevError;
  p->prevError = error;

  return (p->Kp * error) + (p->Ki * p->integral) + (p->Kd * derivative);
}
