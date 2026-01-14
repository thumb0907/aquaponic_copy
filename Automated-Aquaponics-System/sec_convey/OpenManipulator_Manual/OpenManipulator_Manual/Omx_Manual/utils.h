#ifndef UTILS_HPP
#define UTILS_HPP

#include <open_manipulator_libs.h>
#include <Eigen/Dense>
#include <math.h>

OpenManipulator omx;

// ==================== 초기화 ====================
inline void initManipulator()
{
  omx.initOpenManipulator(true);
  Serial.println("[OK] Manipulator initialized.");
}

// ==================== readJoint(): 현재 조인트 각도 읽기 ====================
inline std::vector<double> readJoint()
{
  std::vector<robotis_manipulator::JointValue> joints = omx.getAllActiveJointValue();
  std::vector<double> jnt_deg(4);

  for (int i = 0; i < 4; i++)
    jnt_deg[i] = joints[i].position * 180.0 / M_PI;  // rad → deg 변환

  Serial.println("[INFO] readJointDeg(): Current joint angles (deg)");
  for (int i = 0; i < 4; i++)
  {
    Serial.print("  J"); Serial.print(i + 1);
    Serial.print(": "); Serial.print(jnt_deg[i], 2);
    Serial.println(" deg");
  }
  return jnt_deg;
}

// ==================== readTCP(): 현재 TCP 좌표 읽기 ====================
inline Eigen::Vector3d readTCP()
{
  robotis_manipulator::KinematicPose pose = omx.getKinematicPose("gripper");
  Eigen::Vector3d tcp = pose.position;

  Serial.println("[INFO] readTCP(): Current TCP position (m)");
  Serial.print("  X: "); Serial.println(tcp(0), 6);
  Serial.print("  Y: "); Serial.println(tcp(1), 6);
  Serial.print("  Z: "); Serial.println(tcp(2), 6);
  return tcp;
}

// ==================== moveHome(): 모든 관절을 0 rad 위치로 이동 ====================
inline void moveHome(double t = 2.0)
{
  std::vector<double> home_pos = {0.0, 0.0, 0.0, 0.0};
  omx.makeJointTrajectory(home_pos, t);
  Serial.println("[CMD] moveHome() - Move to moveHome position (0 rad)");
}

// ==================== moveJointAbs(): Joint 절대이동 ====================
inline void moveJointAbs(float j1, float j2, float j3, float j4, double t = 2.0)
{
  // deg → rad 변환
  std::vector<double> goal_rad = {
    j1 * M_PI / 180.0,
    j2 * M_PI / 180.0,
    j3 * M_PI / 180.0,
    j4 * M_PI / 180.0
  };

  omx.makeJointTrajectory(goal_rad, t);
  Serial.println("[CMD] moveJointAbsDeg() - Joint absolute move (deg input).");
}


// ==================== moveJointRel(): Joint 상대이동 ====================
inline void moveJointRel(float dj1, float dj2, float dj3, float dj4, double t = 2.0)
{
  // deg → rad 변환
  std::vector<double> delta_rad = {
    dj1 * M_PI / 180.0,
    dj2 * M_PI / 180.0,
    dj3 * M_PI / 180.0,
    dj4 * M_PI / 180.0
  };

  omx.makeJointTrajectoryFromPresentPosition(delta_rad, t);
  Serial.println("[CMD] moveJointRelDeg() - Joint relative move (deg input).");
}

// ==================== moveTCPAbs(): TCP 절대 위치 직선이동 ====================
inline void moveTCPAbs(float x, float y, float z, double t = 2.0)
{
  Eigen::Vector3d pos(x, y, z);
  omx.makeTaskTrajectory("gripper", pos, t);
  Serial.println("[CMD] moveTCPAbs() - Task absolute move.");
}

// ==================== moveTCPRel(): TCP 상대이동 ====================
inline void moveTCPRel(float dx, float dy, float dz, double t = 2.0)
{
  Eigen::Vector3d delta(dx, dy, dz);
  omx.makeTaskTrajectoryFromPresentPose("gripper", delta, t);
  Serial.println("[CMD] moveTCPRel() - Task relative move.");
}

// ==================== setGripper(): 그리퍼 제어 ====================
inline void setGripper(bool open, double mag = 0.01)
{
  double val = open ? +mag : -mag;
  omx.makeToolTrajectory("gripper", val);
  Serial.println("[CMD] setGripper() - Gripper move.");
}


// ==================== keepHorizontal(): 수평유지 보정 ====================
// 조인트 2, 3, 4의 합이 0이 되도록 조정 -> TCP가 수평을 유지하도록 함.
inline void keepHorizontal(double t = 0.7)
{
  std::vector<double> jnt = readJoint();  // [J1, J2, J3, J4] in deg

  // deg → rad 변환
  double j1 = jnt[0] * M_PI / 180.0;
  double j2 = jnt[1] * M_PI / 180.0;
  double j3 = jnt[2] * M_PI / 180.0;

  // 수평 유지 계산 (rad)
  double j4_new = -(j2 + j3) - 0.05;

  // rad 단위 goal
  std::vector<double> goal = {j1, j2, j3, j4_new};
  omx.makeJointTrajectory(goal, t);

  Serial.println("[CMD] keepHorizontalDeg() - Adjust J4 to maintain horizontal gripper (deg input).");
  Serial.print("  J4 corrected to: "); Serial.print(j4_new * 180.0 / M_PI, 2); Serial.println(" deg");
}

// ==================== setPitch(): 지정한 pitch각으로 기울이기 ====================
// pitch(rad) = J2 + J3 + J4  →  J4 = pitch - (J2 + J3)
inline void setPitch(double target_pitch_deg, double t = 0.7)
{
  // 현재 조인트 각도 읽기 (deg 단위)
  std::vector<double> jnt_deg = readJoint();  // [J1, J2, J3, J4]

  // deg → rad 변환
  double j1 = jnt_deg[0] * M_PI / 180.0;
  double j2 = jnt_deg[1] * M_PI / 180.0;
  double j3 = jnt_deg[2] * M_PI / 180.0;
  double target_pitch_rad = target_pitch_deg * M_PI / 180.0;

  // 지정된 pitch 각도에 맞게 J4 계산
  double j4_new = target_pitch_rad - (j2 + j3);

  // rad 단위 goal 벡터 생성
  std::vector<double> goal = {j1, j2, j3, j4_new};
  omx.makeJointTrajectory(goal, t);

  // 출력
  Serial.print("[CMD] setPitch() - Adjust J4 for target pitch (");
  Serial.print(target_pitch_deg, 2); Serial.println(" deg).");
  Serial.print("  J4 corrected to: ");
  Serial.print(j4_new * 180.0 / M_PI, 2);
  Serial.println(" deg");
}


#endif
