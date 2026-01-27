#ifndef MOVE_H
#define MOVE_H

#include "encoder.h"
#include "set_motor.h"
#include "kinematic.h"

//unsigned long J2_DEFAULT_PP;

void move_j2_mm(float mm, unsigned long pps =  J2_DEFAULT_PPS);
void move_j2_cm(float cm, unsigned long pps =  J2_DEFAULT_PPS);
void home();
void goXY(float x, float y);
void printXY(float th1_deg, float th2_deg);
void goXY_keepParallel(float x, float y);
#endif