#ifndef PID_H
#define PID_H

typedef struct {
  float Kp, Ki, Kd;
  float prevError;
  float integral;
  float iLimit;
} PID;

extern PID j1_pid;
extern PID j2_pid;
extern PID j3_pid;
extern PID j4_pid;

float pid_update(PID *p, float error);

#endif
