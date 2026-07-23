#include "pid.h"

//PID   j1_pid = { 55.0f, 0.0f, 10.0f, 0.0f, 0.0f, 5000.0f };
PID   j1_pid = { 6.5f, 0.1f, 2.5f, 0.0f, 0.0f, 5000.0f };
//PID   j2_pid = { 30.0f, 0.0f, 20.0f, 0.0f, 0.0f, 5000.0f };
//PID   j3_pid = { 40.0f, 0.0f, 10.0f, 0.0f, 0.0f, 5000.0f };
PID   j3_pid = { 20.0f, 0.2f, 17.0f, 0.0f, 0.0f, 5000.0f };
PID   j4_pid = { 15.0f, 0.0f, 12.0f, 0.0f, 0.0f, 5000.0f };

float pid_update(PID *p, float error) {
  p->integral += error;
  if (p->integral >  p->iLimit) p->integral =  p->iLimit;
  if (p->integral < -p->iLimit) p->integral = -p->iLimit;

  float derivative = error - p->prevError;
  p->prevError = error;

  return (p->Kp * error) + (p->Ki * p->integral) + (p->Kd * derivative);
}
