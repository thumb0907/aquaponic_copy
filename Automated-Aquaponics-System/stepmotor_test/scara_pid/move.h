#ifndef MOVE_H
#define MOVE_H

#include "encoder.h"
#include "set_motor.h"
#include "kinematic.h"
#include <math.h>
#include "servo_config.h"

//unsigned long J2_DEFAULT_PP;
void move_j2_mm(float mm, unsigned long pps =  J2_DEFAULT_PPS);
void move_j2_cm(float cm, unsigned long pps =  J2_DEFAULT_PPS);
void home();
void goXY(float x, float y, unsigned long pps1, unsigned long int pps2);
//void printXY(float th1_deg, float th2_deg);
//void goXY_keepParallel(float x, float y);
void tool(bool on);
//void moveXY_rel(float dx_mm, float dy_mm);

// ---- Joint move (ABS/REL) ----
void moveJ_abs(float th1_deg, float th3_deg);
void moveJ_rel(float dth1_deg, float dth3_deg);
void moveJ_abs4(float th1_deg, float th3_deg, float th4_deg);
void moveJ_rel4(float dth1_deg, float dth3_deg, float dth4_deg);
static bool move_sync_13(float th1_deg, float th3_deg,
                         unsigned long j1_max_pps,
                         unsigned long j3_max_pps,
                         float tolDeg,
                         unsigned long stable_ms,
                         unsigned long timeout_ms);
// ---- IK best-solution (elbow auto) ----
bool inverse2R_best(float x, float y,
                    float th1_cur_deg, float th2_cur_deg,
                    float& th1_out_deg, float& th2_out_deg);

void moveRail(unsigned long pps, bool dir);
void stopRail(bool disable_after = false);


void grip(bool st);
#endif