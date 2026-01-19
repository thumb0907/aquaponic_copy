#include "move.h"
#include "encoder.h"
#include "kinematic.h"

unsigned long t1=0, t2=0, t3=0, t4=0;
const unsigned long PID_DT_MS = 10;

void move(float j1, float j2, float j3, float j4)
{
  unsigned long now = millis();
  if (now - t1 >= PID_DT_MS) { t1 = now; move_j1(j1); }
  //if (now - t2 >= PID_DT_MS) { t2 = now; move_j2(j2); }
  if (now - t3 >= PID_DT_MS) { t3 = now; move_j3(j3); }
  if (now - t4 >= PID_DT_MS) { t4 = now; move_j4(j4); }
}