#include "move.h"

//Servo myservo;
// =========================
static void stop_joints_134()
{
  j1_run = false;
  j3_run = false;
  j4_run = false;

  digitalWrite(j1_pul, LOW);
  digitalWrite(j3_pul, LOW);
  digitalWrite(j4_pul, LOW);
}
static bool move_sync_13(float th1_deg, float th3_deg,
                         unsigned long j1_max_pps,
                         unsigned long j3_max_pps,
                         float tolDeg,
                         unsigned long stable_ms,
                         unsigned long timeout_ms)
{
  motors_enable_all(true);

  unsigned long t0 = millis();
  unsigned long inTolSince = 0;

  while (true) {
    bool done1 = move_j1(th1_deg, j1_max_pps, tolDeg);
    bool done3 = move_j3(th3_deg, j3_max_pps, tolDeg);

    if (done1 && done3) {
      if (inTolSince == 0) inTolSince = millis();

      if (millis() - inTolSince >= stable_ms) {
        stop_joints_134();
        motors_enable_all(false);
        return true;
      }
    } else {
      inTolSince = 0;
    }

    if (millis() - t0 > timeout_ms) {
      Serial.println("[SYNC13] timeout");
      stop_joints_134();
      motors_enable_all(false);
      return false;
    }
  }
}
static bool move_sync_134(float th1_deg, float th3_deg, float th4_deg,
                          unsigned long timeout_ms = 8000)
{
  motors_enable_all(true);

  unsigned long t0 = millis();
  bool done1 = false;
  bool done3 = false;
  bool done4 = false;

  while (true) {
    if (!done1) done1 = move_j1(th1_deg);
    if (!done3) done3 = move_j3(th3_deg);
    if (!done4) done4 = move_j4(th4_deg);

    if (done1 && done3 && done4) {
      stop_joints_134();
      motors_enable_all(false);
      return true;
    }

    if (millis() - t0 > timeout_ms) {
      Serial.println("[SYNC134] timeout");
      stop_joints_134();
      motors_enable_all(false);
      return false;
    }

    //delay(2);
  }
}

// =========================

// =========================
void move_j2_mm(float mm, unsigned long pps)
{
  float deg = (mm / J2_LEAD_MM_PER_REV) * 360.0f;
  move_j2(deg, pps);
}

void move_j2_cm(float cm, unsigned long pps)
{
  move_j2_mm(cm * 10.0f, pps);
}

void home()
{
  motors_enable_all(true);
  delay(100);
  //grip_timer_off(); 
  j4_home();
  delay(100);
  //j4_stop();
  //Serial.println("4 fin");
  //delay(1000);

  grip(false);
  j2_home_stop_on_switch(true, 4000);   // 또는 j2_home_precise(true, 4000, 1000);
  delay(50);
  Serial.println("j2 end");
  //delay(1000);

  //delay(50);
  enc_reset_j3();
  move_j3_wait(170, 5500);
  enc_reset_j3();
  delay(100);
  //Serial.println("j3 back");
  j3_home_stop_on_switch(true, 4000);
  enc_reset_j3();
  delay(5);
  //Serial.println("j3 home");
  move_j3_wait(-90);  
  enc_reset_j3();

  move_j2_cm(-1.0f, 4000);
  Serial.println("j2 home");

  enc_reset_j1();
  delay(10);
  //Serial.println("j1_back start");
  move_j1_wait(-30);
  //Serial.println("j1_back");
  delay(200);

  j1_home_stop_on_switch(false, 2000);
  delay(500);
  enc_reset_j1();
  //Serial.println("j1 home");
  //delay(1000);

  move_j1_wait(-40);
  enc_reset_j1();
  delay(100);
  //Serial.println("j1 end");
/*
  j4_home_stop_on_switch_safe(false, 2000);
  delay(500);
  move_j4_wait(-15.0f);
  delay(50);
*/
  //j4_home_openloop(true, 2300);   // 홈 스위치로 원점
  //delay(200);

  //move_j4_openloop(15.0f, 1200);  // 절대각 -15도
  //j4_home_openloop(true, 2000);
  delay(200);

  moveRail(false, 4000);
  delay(300);
  stopRail();
  delay(100);

  moveRail_untilStop(true, 4300, stop_rail);
  delay(100);

  enc_reset_all();
  motors_enable_all(false);
}

void goXY(float x, float y, unsigned long pps1, unsigned long int pps2)
{
  float th1_cur = j1_getJointDeg();
  float th2_cur = j3_getJointDeg();

  float th1, th2;
  bool ok = inverse2R_best(x, y, th1_cur, th2_cur, th1, th2);
  if (!ok) {
    Serial.println("[IK] unreachable");
    return;
  }

  //move_sync_13(th1, -th2, pps1, pps2);
}

void printXY(float th1_deg, float th2_deg)
{
  Pose2D p = forward2R(th1_deg, th2_deg, L1_mm, L2_mm);
  Serial.print("X="); Serial.print(p.x_mm);
  Serial.print("  Y="); Serial.println(p.y_mm);
}

void moveXY_rel(float dx_mm, float dy_mm)
{
  float th1_cur = j1_getJointDeg();
  float th2_cur = j3_getJointDeg();

  Pose2D cur = forward2R(th1_cur, th2_cur, L1_mm, L2_mm);

  float x_tgt = cur.x_mm + dx_mm;
  float y_tgt = cur.y_mm + dy_mm;

  float th1_tgt, th2_tgt;
  if (!inverse2R_best(x_tgt, y_tgt, th1_cur, th2_cur, th1_tgt, th2_tgt)) {
    Serial.println("[IK] unreachable (rel)");
    return;
  }

  //move_sync_13(th1_tgt, th2_tgt);
}

void goXY_keepParallel(float x, float y)
{
  float th1_cur = j1_getJointDeg();
  float th2_cur = j3_getJointDeg();

  float th1, th2;
  if (!inverse2R_best(x, y, th1_cur, th2_cur, th1, th2)) {
    Serial.println("[IK] unreachable");
    return;
  }

  float phi_offset = 0.0f;
  float wrist_deg = wristPhiParallelX(th1, th2, phi_offset);

  move_sync_134(th1, th2, wrist_deg);
}

void tool(bool on)
{
  if (on) {
    digitalWrite(num1, HIGH);
    digitalWrite(num2, LOW);
    digitalWrite(num3, HIGH);
  } else {
    digitalWrite(num1, LOW);
    digitalWrite(num2, HIGH);
    digitalWrite(num3, LOW);
  }
}

void moveJ_abs(float th1_deg, float th3_deg)
{
  //move_sync_13(th1_deg, th3_deg);
}

void moveJ_rel(float dth1_deg, float dth3_deg)
{
  float th1_cur = j1_getJointDeg();
  float th3_cur = j3_getJointDeg();

  moveJ_abs(th1_cur + dth1_deg, th3_cur + dth3_deg);
}

void moveJ_abs4(float th1_deg, float th3_deg, float th4_deg)
{
  move_sync_134(th1_deg, th3_deg, th4_deg);
}

void moveJ_rel4(float dth1_deg, float dth3_deg, float dth4_deg)
{
  float th1_cur = j1_getJointDeg();
  float th3_cur = j3_getJointDeg();
  float th4_cur = j4_getJointDeg();

  moveJ_abs4(th1_cur + dth1_deg, th3_cur + dth3_deg, th4_cur + dth4_deg);
}

