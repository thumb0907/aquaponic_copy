#ifndef UTILS_HPP
#define UTILS_HPP

#include "pen_manipulator.h"
#include <Eigen/Dense>
#include <math.h>

PenManipulator omx;

#define PEN_OFFSET_Z 0.043

// ==================== 초기화 ====================
inline void initManipulator()
{
  omx.initPenManipulator(true);
  Serial.println("[OK] Manipulator initialized (Pen version).");
}

// ==================== readJoint(): 현재 조인트 각도 읽기 ====================
inline std::vector<double> readJoint()
{
  std::vector<robotis_manipulator::JointValue> joints = omx.getAllActiveJointValue();
  std::vector<double> jnt_deg(joints.size());

  for (int i = 0; i < (int)joints.size(); i++)
    jnt_deg[i] = joints[i].position * 180.0 / M_PI;  // rad to deg 변환

  Serial.println("[INFO] readJoint(): Current joint angles (deg)");
  for (int i = 0; i < (int)joints.size(); i++)
  {
    Serial.print("  J"); Serial.print(i + 1);
    Serial.print(": "); Serial.print(jnt_deg[i], 2);
    Serial.println(" deg");
  }
  return jnt_deg;
}

// ==================== readTCP(): 펜 끝 TCP 좌표 ====================
inline Eigen::Vector3d readTCP()
{
  robotis_manipulator::KinematicPose pose = omx.getKinematicPose("pen");
  Eigen::Vector3d tcp = pose.position;

  Eigen::Matrix3d R = pose.orientation;
  tcp += R * Eigen::Vector3d(0, 0, PEN_OFFSET_Z);

  Serial.println("[INFO] readTCP(): Current TCP (pen tip) position (m)");
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

// ==================== moveJointAbs(): Joint 절대이동 (deg 단위 입력) ====================
inline void moveJointAbs(float j1_deg, float j2_deg, float j3_deg, float j4_deg, double t = 2.0)
{
  // deg → rad 변환
  std::vector<double> goal_rad = {
    j1_deg * M_PI / 180.0,
    j2_deg * M_PI / 180.0,
    j3_deg * M_PI / 180.0,
    j4_deg * M_PI / 180.0
  };

  omx.makeJointTrajectory(goal_rad, t);
  Serial.println("[CMD] moveJointAbs() - Joint absolute move (deg input).");
}

// ==================== moveJointRel(): Joint 상대이동 (deg 단위 입력) ====================
inline void moveJointRel(float dj1_deg, float dj2_deg, float dj3_deg, float dj4_deg, double t = 2.0)
{
  // deg → rad 변환
  std::vector<double> delta_rad = {
    dj1_deg * M_PI / 180.0,
    dj2_deg * M_PI / 180.0,
    dj3_deg * M_PI / 180.0,
    dj4_deg * M_PI / 180.0
  };

  omx.makeJointTrajectoryFromPresentPosition(delta_rad, t);
  Serial.println("[CMD] moveJointRel() - Joint relative move (deg input).");
}

// ==================== moveTCPAbs(): TCP 절대 위치 직선이동 ====================
inline void moveTCPAbs(float x, float y, float z, double t = 2.0)
{
  // 입력값은 펜 팁 좌표 (TCP 기준)
  Eigen::Vector3d tcp_tip(x, y, z);

  // 현재 펜의 orientation 얻기
  robotis_manipulator::KinematicPose pose = omx.getKinematicPose("pen");
  Eigen::Matrix3d R = pose.orientation;

  // 펜 중심으로 보정 (pen origin = pen tip - R*(0,0,PEN_OFFSET_Z))
  Eigen::Vector3d pen_origin = tcp_tip - R * Eigen::Vector3d(0, 0, PEN_OFFSET_Z);

  // Trajectory 생성 (기준: pen link origin)
  omx.makeTaskTrajectory("pen", pen_origin, t);

  Serial.println("[CMD] moveTCPAbs() - Task absolute move (tip→origin corrected).");
}


// ==================== moveTCPRel(): TCP 상대이동 ====================
inline void moveTCPRel(float dx, float dy, float dz, double t = 2.0)
{
  Eigen::Vector3d delta(dx, dy, dz);
  omx.makeTaskTrajectoryFromPresentPose("pen", delta, t);
  Serial.println("[CMD] moveTCPRel() - Task relative move.");
}

// ==================== keepHorizontal(): 수평 유지 ====================
inline void keepHorizontal(double t = 0.7)
{
  std::vector<double> jnt_deg = readJoint();  // [J1, J2, J3, J4] in degree

  // deg → rad 변환
  double j1 = jnt_deg[0] * M_PI / 180.0;
  double j2 = jnt_deg[1] * M_PI / 180.0;
  double j3 = jnt_deg[2] * M_PI / 180.0;

  // 수평 보정 (rad 단위)
  double j4_new = -(j2 + j3) - 0.05; // 0.05 rad 보정

  // rad 단위 goal
  std::vector<double> goal = {j1, j2, j3, j4_new};
  omx.makeJointTrajectory(goal, t);

  Serial.println("[CMD] keepHorizontal() - Adjust J4 to maintain horizontal pen.");
}

// ==================== setPitch(): 지정한 pitch각으로 기울이기 ====================
inline void setPitch(double target_pitch_deg, double t = 0.7)
{
  std::vector<double> jnt_deg = readJoint();  // [J1, J2, J3, J4] in degree

  // deg → rad 변환
  double j1 = jnt_deg[0] * M_PI / 180.0;
  double j2 = jnt_deg[1] * M_PI / 180.0;
  double j3 = jnt_deg[2] * M_PI / 180.0;
  double target_pitch_rad = target_pitch_deg * M_PI / 180.0;

  // pitch 각도에 맞게 J4 계산
  double j4_new = target_pitch_rad - (j2 + j3);

  // rad 단위 goal
  std::vector<double> goal = {j1, j2, j3, j4_new};
  omx.makeJointTrajectory(goal, t);

  Serial.print("[CMD] setPitch() - Adjust J4 for target pitch (");
  Serial.print(target_pitch_deg, 2);
  Serial.println(" deg).");
}


#endif
