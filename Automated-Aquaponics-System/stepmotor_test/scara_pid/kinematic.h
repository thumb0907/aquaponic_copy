#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <Arduino.h>

struct Pose2D {
  float x_mm;
  float y_mm;
};

struct Joint2R {
  float th1_deg;
  float th2_deg;
};

// Forward: (th1, th2) -> (x, y)
Pose2D forward2R(float th1_deg, float th2_deg, float L1_mm, float L2_mm);

// Inverse: (x, y) -> (th1, th2)
// elbowUp=true/false 로 해 선택 가능
bool inverse2R(float x_mm, float y_mm, float L1_mm, float L2_mm,
               bool elbowUp,
               float &th1_deg, float &th2_deg);

// 그리퍼를 X축에 평행하게 유지하고 싶을 때(필요하면 사용)
float wristPhiParallelX(float th1_deg, float th2_deg, float phi_offset_deg);

#endif
