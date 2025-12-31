#ifndef UTILS_HPP
#define UTILS_HPP

#include <open_manipulator_libs.h>
#include <Eigen/Dense>
#include <math.h>
#include <vector>

extern OpenManipulator omx;                           // 객체 소환

void initManipulator();                        // 초기화 함수
void runManipulator(double sec);               // 내부 제어 루프 실행
std::vector<double> readJoint();               // 현재 조인트 각도 읽기
Eigen::Vector3d readTCP();                     // TCP좌표 읽기
void moveHome(double t = 2.0);                 // 홈 이동
void moveJointAbs(float j1, float j2,          // 조인트 기준 절대 이동
                   float j3, float j4,
                    double t = 2.0);             
void moveJointRel(float dj1, float dj2,        // 조인트 기준 상대 이동
                   float dj3, float dj4,
                    double t = 2.0);      
void moveTCPAbs(float x, float y, float z,     // TPC기준 절대 이동(m)
                double t = 2.0);
void moveTCPRel(float dx, float dy, float dz,  // TCP기준 상대 이동(m)
                 double t = 2.0);
void setGripper(bool open, double mag = 0.01,  // 그리퍼 on/off
                double t = 1.0);
void keepHorizontal(double t = 2);             // TCP 수평조정          
void setPitch(double target_pitch_deg,         //pitch 각도 조정
                     double t = 1.5);

#endif