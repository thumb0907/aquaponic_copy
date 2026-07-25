#pragma once

#include <Arduino.h>
#include <open_manipulator_libs.h>
#include <DynamixelWorkbench.h>

struct JointPose
{
  double j1;
  double j2;
  double j3;
  double j4;
};

extern OpenManipulator omx;
extern DynamixelWorkbench gripper_wb;

bool initManipulator();

void processManipulatorOnce();
bool runManipulator(double seconds);  

bool movePoseTimed(
  const JointPose &pose,
  double move_time
);

bool movePoseAtSpeed(
  const JointPose &pose,
  double speed_rad_s
);

bool openGripper();
bool closeGripper();