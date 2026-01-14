#ifndef PEN_MANIPULATOR_HPP
#define PEN_MANIPULATOR_HPP

#include <open_manipulator_libs.h>

using namespace robotis_manipulator;

#define JOINT_DYNAMIXEL "joint_dxl"
#define X_AXIS math::vector3(1.0, 0.0, 0.0)
#define Y_AXIS math::vector3(0.0, 1.0, 0.0)
#define Z_AXIS math::vector3(0.0, 0.0, 1.0)

// ----------------------------
// PenManipulator 클래스 정의
// ----------------------------
class PenManipulator : public RobotisManipulator
{
private:
  Kinematics* kinematics_;
  JointActuator* joint_actuator_;

public:
  PenManipulator() {}
  virtual ~PenManipulator()
  {
    delete kinematics_;
    delete joint_actuator_;
  }

  void initPenManipulator(bool using_actual_robot_state,
                          STRING usb_port = "/dev/ttyUSB0",
                          STRING baud_rate = "1000000",
                          float control_loop_time = 0.010)
  {
    /**************************************************************************
     * 링크 구조 정의 (4 DOF + pen)
     **************************************************************************/
    addWorld("world", "joint1");

    addJoint("joint1", "world", "joint2",
             math::vector3(0.012, 0.0, 0.017),
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
             Z_AXIS, 11, M_PI, -M_PI);

    addJoint("joint2", "joint1", "joint3",
             math::vector3(0.0, 0.0, 0.0595),
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
             Y_AXIS, 12, M_PI_2, -2.05);

    addJoint("joint3", "joint2", "joint4",
             math::vector3(0.024, 0.0, 0.128),
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
             Y_AXIS, 13, 1.53, -M_PI_2);

    addJoint("joint4", "joint3", "pen",
             math::vector3(0.124, 0.0, 0.0),
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
             Y_AXIS, 14, 2.0, -1.8);

    addTool("pen", "joint4",
            math::vector3(0.043, 0.0, 0.0),
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
            -1);

    /**************************************************************************
     * 기구학 설정
     **************************************************************************/
    kinematics_ = new kinematics::SolverCustomizedforOMChain();
    addKinematics(kinematics_);

    /**************************************************************************
     * Dynamixel 설정
     **************************************************************************/
    if (using_actual_robot_state)
    {
      joint_actuator_ = new dynamixel::JointDynamixelProfileControl(control_loop_time);

      STRING dxl_comm_arg[2] = {usb_port, baud_rate};
      void* p_dxl_comm_arg = &dxl_comm_arg;

      std::vector<uint8_t> joint_ids = {11, 12, 13, 14};
      addJointActuator(JOINT_DYNAMIXEL, joint_actuator_, joint_ids, p_dxl_comm_arg);

      STRING mode_arg = "position_mode";
      void* p_mode_arg = &mode_arg;
      setJointActuatorMode(JOINT_DYNAMIXEL, joint_ids, p_mode_arg);

      STRING gain_arg[2] = {"Position_P_Gain", "650"}; // gain값 해보면서 계속 보정
      void* p_gain_arg = &gain_arg;
      setJointActuatorMode(JOINT_DYNAMIXEL, joint_ids, p_gain_arg);

      enableAllActuator();
      receiveAllJointActuatorValue();
    }

    log::println("[OK] PenManipulator initialized (4 DOF, pen tool).");
  }

  // ---------------------------
  // 주기 처리 루프
  // ---------------------------
  void processPenManipulator(double present_time)
  {
    JointWaypoint goal_joint_value = getJointGoalValueFromTrajectory(present_time);

    receiveAllJointActuatorValue();
    if (goal_joint_value.size() != 0)
      sendAllJointActuatorValue(goal_joint_value);
    solveForwardKinematics();
  }
};

#endif
