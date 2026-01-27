#include "move.h"

void move_j2_mm(float mm, unsigned long pps)
{
  // mm -> 회전수 -> deg
  float deg = (mm / J2_LEAD_MM_PER_REV) * 360.0f;
  move_j2(deg, pps);   // 기존 함수 그대로 활용
}

// cm 단위 이동
void move_j2_cm(float cm, unsigned long pps)
{
  move_j2_mm(cm * 10.0f, pps);
}

void home()
{
  //move_j1_wait(90.0f);
  //delay(50);
  j1_home_stop_on_switch(true, 2000);
  delay(500);
  enc_reset_j1();
  //delay(50);
  move_j1_wait(-180.0f);
  delay(50);

  bool ok = move_j2(7200.0f, 600000);

  if (!ok) {
    move_j2_mm(-10.0f);
  }
  move_j2_cm(-5.0f);

  move_j3_wait(200);
  delay(50);
  j3_home_stop_on_switch(true, 3000);
  delay(500);
  enc_reset_j3();
  move_j3_wait(-20.0f);
  delay(50);

  //move_j4_wait(20);
  //delay(50);
  j4_home_stop_on_switch_safe(true, 2000);
  delay(500);
  enc_reset_j4();
  //move_j4_wait(-15.0f);
  //delay(50);
}


void goXY(float x, float y)
{
  float th1, th2;

  bool ok = inverse2R(x, y, L1_mm, L2_mm, /*elbowUp=*/true, th1, th2);
  if (!ok) {
    Serial.println("[IK] unreachable");
    return;
  }

  // 이제 조인트각으로 이동
  move_j1_wait(th1);
  move_j3_wait(th2);
  Serial.println(th2);
}

void printXY(float th1_deg, float th2_deg)
{
  Pose2D p = forward2R(th1_deg, th2_deg, L1_mm, L2_mm);
  Serial.print("X="); Serial.print(p.x_mm);
  Serial.print("  Y="); Serial.println(p.y_mm);
}

void goXY_keepParallel(float x, float y)
{
  float th1, th2;
  if (!inverse2R(x, y, L1_mm, L2_mm, true, th1, th2)) return;

  float phi_offset = 0.0f; // 기구 조립 기준에 따라 보정값 필요
  float wrist_deg = wristPhiParallelX(th1, th2, phi_offset);

  move_j1_wait(th1);
  move_j3_wait(th2);
  move_j4_wait(wrist_deg);   // 너 프로젝트에 j4가 있다면
}

