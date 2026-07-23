#include "kinematic.h"
#include <math.h>

float L1_mm = 240;
float L2_mm = 160;

Pose2D forward2R(float th1_deg, float th2_deg, float L1_mm, float L2_mm) {
  float t1 = th1_deg * DEG_TO_RAD;
  float t2 = th2_deg * DEG_TO_RAD;

  Pose2D p;
  p.x_mm = L1_mm * cosf(t1) + L2_mm * cosf(t1 + t2);
  p.y_mm = L1_mm * sinf(t1) + L2_mm * sinf(t1 + t2);
  return p;
}
//함수 사용 : Pose2D forward2R(float th1_deg, float ...);

bool inverse2R(float x_mm, float y_mm, float L1_mm, float L2_mm,
               bool elbowUp,
               float &th1_deg, float &th2_deg)
{
  float r2 = x_mm*x_mm + y_mm*y_mm;
  float r  = sqrtf(r2);

  // 도달 가능 영역 체크
  if (r > (L1_mm + L2_mm)) return false;
  if (r < fabsf(L1_mm - L2_mm)) return false;

  float c2 = (r2 - L1_mm*L1_mm - L2_mm*L2_mm) / (2.0f*L1_mm*L2_mm);

  // 부동소수 오차 clamp
  if (c2 >  1.0f) c2 =  1.0f;
  if (c2 < -1.0f) c2 = -1.0f;

  float s2 = sqrtf(fmaxf(0.0f, 1.0f - c2*c2));
  if (elbowUp) s2 = -s2;

  float th2 = atan2f(s2, c2);
  float th1 = atan2f(y_mm, x_mm) - atan2f(L2_mm*s2, L1_mm + L2_mm*c2);

  th1_deg = th1 * RAD_TO_DEG;
  th2_deg = th2 * RAD_TO_DEG;
  //th2_deg = -th2_deg;
  
  return true;
}

float wristPhiParallelX(float th1_deg, float th2_deg, float phi_offset_deg) {
  // “그리퍼를 X축과 평행” 같은 조건을 단순화한 형태(필요한 offset은 기구에 맞게)
  return -(th1_deg + th2_deg) + phi_offset_deg;
}

static inline float sqr(float x) { return x * x; }

bool inverse2R_best(float x, float y,
                    float th1_cur_deg, float th2_cur_deg,
                    float& th1_out_deg, float& th2_out_deg)
{
  float th1_u, th2_u;
  float th1_d, th2_d;

  bool ok_u = inverse2R(x, y, L1_mm, L2_mm, /*elbowUp=*/true,  th1_u, th2_u);
  bool ok_d = inverse2R(x, y, L1_mm, L2_mm, /*elbowUp=*/false, th1_d, th2_d);

  if (!ok_u && !ok_d) return false;

  // 둘 다 가능하면 "현재 자세와 가장 가까운 해"를 선택
  if (ok_u && ok_d) {
    float du = sqr(th1_u - th1_cur_deg) + sqr(th2_u - th2_cur_deg);
    float dd = sqr(th1_d - th1_cur_deg) + sqr(th2_d - th2_cur_deg);
    if (du <= dd) { th1_out_deg = th1_u; th2_out_deg = th2_u; }
    else          { th1_out_deg = th1_d; th2_out_deg = th2_d; }
    return true;
  }

  // 하나만 가능하면 그 해 사용
  if (ok_u) { th1_out_deg = th1_u; th2_out_deg = th2_u; return true; }
  else      { th1_out_deg = th1_d; th2_out_deg = th2_d; return true; }
}

