#include "utils.h"

#include <math.h>
#include <vector>

OpenManipulator omx;
DynamixelWorkbench gripper_wb;

// =====================================================
// ID 15 그리퍼 설정
// =====================================================

static constexpr char GRIPPER_DEVICE_NAME[] = "";
static constexpr uint32_t GRIPPER_BAUDRATE = 1000000;
static constexpr uint8_t GRIPPER_ID = 15;

// 현재 정상 동작하는 그리퍼 설정 유지
static constexpr float GRIPPER_OPEN_MOTOR_RAD = 1.50f;
static constexpr float GRIPPER_CLOSE_MOTOR_RAD = -0.6666667f;

static constexpr int32_t GRIPPER_CURRENT = 200;
static constexpr int32_t GRIPPER_PROFILE_ACCELERATION = 20;
static constexpr int32_t GRIPPER_PROFILE_VELOCITY = 250;

static constexpr int32_t GRIPPER_POSITION_TOLERANCE = 20;
static constexpr int32_t GRIPPER_MIN_CLOSE_TRAVEL = 80;
static constexpr int32_t GRIPPER_STALL_POSITION_DELTA = 3;

static constexpr unsigned long GRIPPER_TIMEOUT_MS = 4000;
static constexpr unsigned long GRIPPER_COMMAND_REPEAT_MS = 200;
static constexpr unsigned long GRIPPER_PRINT_INTERVAL_MS = 250;
static constexpr unsigned long GRIPPER_CONTACT_STALL_MS = 350;

// =====================================================
// 관절 이동 설정
// =====================================================

static constexpr double DEFAULT_SPEED_RAD_S = 0.42;
static constexpr double MIN_MOVE_TIME = 0.80;
static constexpr double MAX_MOVE_TIME = 15.0;

// 목표 자세 도달 확인
static constexpr double JOINT_REACHED_TOLERANCE_RAD = 0.12;
static constexpr unsigned long JOINT_REACHED_TIMEOUT_MS = 2500;
static constexpr unsigned long JOINT_VERIFY_INTERVAL_MS = 60;

static bool gripper_ready = false;

static int32_t gripper_open_position = 0;
static int32_t gripper_close_position = 0;

static std::vector<double> joint_goal(4, 0.0);

// =====================================================
// 그리퍼 읽기/쓰기
// =====================================================

static bool readGripperPosition(
  int32_t &position,
  bool print_error
)
{
  if (!gripper_ready)
  {
    if (print_error)
    {
      Serial.println("[GRIPPER] NOT READY");
    }

    return false;
  }

  const char *log = nullptr;

  const bool result =
    gripper_wb.itemRead(
      GRIPPER_ID,
      "Present_Position",
      &position,
      &log
    );

  if (!result && print_error)
  {
    Serial.print(
      "[GRIPPER] POSITION READ FAILED: "
    );

    Serial.println(
      log ? log : "unknown"
    );
  }

  return result;
}

static bool writeGripperPosition(
  int32_t position,
  bool print_error
)
{
  if (!gripper_ready)
  {
    if (print_error)
    {
      Serial.println("[GRIPPER] NOT READY");
    }

    return false;
  }

  const char *log = nullptr;

  const bool result =
    gripper_wb.goalPosition(
      GRIPPER_ID,
      position,
      &log
    );

  if (!result && print_error)
  {
    Serial.print(
      "[GRIPPER] GOAL WRITE FAILED: "
    );

    Serial.println(
      log ? log : "unknown"
    );
  }

  return result;
}

// =====================================================
// 현재 관절 위치 읽기
// =====================================================

static bool readCurrentJointValues(
  std::vector<
    robotis_manipulator::JointValue
  > &present
)
{
  present =
    omx.receiveAllJointActuatorValue();

  if (present.size() != 4)
  {
    Serial.print(
      "[ARM] JOINT READ FAILED, SIZE="
    );

    Serial.println(
      present.size()
    );

    return false;
  }

  for (uint8_t i = 0; i < 4; ++i)
  {
    const double value =
      present[i].position;

    // NaN 검사
    if (value != value)
    {
      Serial.print(
        "[ARM] JOINT VALUE IS NaN: J"
      );

      Serial.println(i + 1);

      return false;
    }

    // 무한대 및 비정상적으로 큰 값 검사
    if (
      value > 1000.0 ||
      value < -1000.0
    )
    {
      Serial.print(
        "[ARM] INVALID JOINT VALUE: J"
      );

      Serial.print(i + 1);
      Serial.print(" = ");
      Serial.println(value, 6);

      return false;
    }
  }

  return true;
}

static double getPoseError(
  const JointPose &pose,
  const std::vector<
    robotis_manipulator::JointValue
  > &present
)
{
  double max_error =
    fabs(
      pose.j1 -
      present[0].position
    );

  const double error_j2 =
    fabs(
      pose.j2 -
      present[1].position
    );

  const double error_j3 =
    fabs(
      pose.j3 -
      present[2].position
    );

  const double error_j4 =
    fabs(
      pose.j4 -
      present[3].position
    );

  if (error_j2 > max_error)
  {
    max_error = error_j2;
  }

  if (error_j3 > max_error)
  {
    max_error = error_j3;
  }

  if (error_j4 > max_error)
  {
    max_error = error_j4;
  }

  return max_error;
}

static double calculateMoveTimeFromPresent(
  const JointPose &target,
  double speed_rad_s,
  const std::vector<
    robotis_manipulator::JointValue
  > &present
)
{
  if (speed_rad_s <= 0.0)
  {
    speed_rad_s =
      DEFAULT_SPEED_RAD_S;
  }

  const double max_delta =
    getPoseError(
      target,
      present
    );

  double move_time =
    max_delta /
    speed_rad_s;

  if (move_time < MIN_MOVE_TIME)
  {
    move_time = MIN_MOVE_TIME;
  }

  if (move_time > MAX_MOVE_TIME)
  {
    move_time = MAX_MOVE_TIME;
  }

  return move_time;
}

// =====================================================
// 자세 도달 확인
// =====================================================

static bool waitUntilPoseReached(
  const JointPose &pose
)
{
  const unsigned long start_ms =
    millis();

  while (
    millis() - start_ms <
    JOINT_REACHED_TIMEOUT_MS
  )
  {
    processManipulatorOnce();

    std::vector<
      robotis_manipulator::JointValue
    > present;

    if (!readCurrentJointValues(present))
    {
      return false;
    }

    const double max_error =
      getPoseError(
        pose,
        present
      );

    if (
      max_error <=
      JOINT_REACHED_TOLERANCE_RAD
    )
    {
      return true;
    }

    delay(
      JOINT_VERIFY_INTERVAL_MS
    );
  }

  std::vector<
    robotis_manipulator::JointValue
  > final_position;

  if (readCurrentJointValues(
        final_position
      ))
  {
    Serial.print(
      "[ARM] TARGET NOT REACHED, ERROR="
    );

    Serial.println(
      getPoseError(
        pose,
        final_position
      ),
      4
    );
  }

  return false;
}

// =====================================================
// 그리퍼 명령
// =====================================================

static bool commandGripperAndWait(
  int32_t target_position,
  const char *command_name,
  bool allow_contact_stop
)
{
  int32_t start_position = 0;

  if (!readGripperPosition(
        start_position,
        true
      ))
  {
    return false;
  }

  Serial.print("[GRIPPER] ");
  Serial.print(command_name);
  Serial.print(" START=");
  Serial.print(start_position);
  Serial.print(" TARGET=");
  Serial.println(target_position);

  if (!writeGripperPosition(
        target_position,
        true
      ))
  {
    return false;
  }

  const int32_t start_error =
    labs(
      target_position -
      start_position
    );

  const unsigned long start_ms =
    millis();

  unsigned long last_command_ms = 0;
  unsigned long last_print_ms = 0;
  unsigned long last_motion_ms = start_ms;

  int32_t present_position =
    start_position;

  int32_t previous_position =
    start_position;

  while (
    millis() - start_ms <
    GRIPPER_TIMEOUT_MS
  )
  {
    processManipulatorOnce();

    const unsigned long now_ms =
      millis();

    if (
      now_ms - last_command_ms >=
      GRIPPER_COMMAND_REPEAT_MS
    )
    {
      last_command_ms = now_ms;

      writeGripperPosition(
        target_position,
        false
      );
    }

    if (!readGripperPosition(
          present_position,
          true
        ))
    {
      return false;
    }

    if (
      labs(
        present_position -
        previous_position
      ) >= GRIPPER_STALL_POSITION_DELTA
    )
    {
      previous_position =
        present_position;

      last_motion_ms =
        now_ms;
    }

    const int32_t error =
      labs(
        target_position -
        present_position
      );

    const int32_t moved_toward_target =
      start_error - error;

    if (
      now_ms - last_print_ms >=
      GRIPPER_PRINT_INTERVAL_MS
    )
    {
      last_print_ms = now_ms;

      Serial.print("[GRIPPER] ");
      Serial.print(command_name);
      Serial.print(" PRESENT=");
      Serial.print(present_position);
      Serial.print(" TARGET=");
      Serial.print(target_position);
      Serial.print(" ERROR=");
      Serial.println(error);
    }

    if (
      error <=
      GRIPPER_POSITION_TOLERANCE
    )
    {
      Serial.print("[GRIPPER] ");
      Serial.print(command_name);
      Serial.println(" COMPLETE");

      return true;
    }

    if (
      allow_contact_stop &&
      moved_toward_target >=
        GRIPPER_MIN_CLOSE_TRAVEL &&
      now_ms - last_motion_ms >=
        GRIPPER_CONTACT_STALL_MS
    )
    {
      Serial.print("[GRIPPER] ");
      Serial.print(command_name);
      Serial.print(
        " CONTACT HOLD, PRESENT="
      );

      Serial.println(
        present_position
      );

      writeGripperPosition(
        target_position,
        false
      );

      return true;
    }

    delay(20);
  }

  Serial.print("[GRIPPER] ");
  Serial.print(command_name);
  Serial.print(" FAILED, PRESENT=");
  Serial.print(present_position);
  Serial.print(" TARGET=");
  Serial.println(target_position);

  return false;
}

// =====================================================
// 초기화
// =====================================================

bool initManipulator()
{
  Serial.println("[OMX] INIT START");

  omx.initOpenManipulator(true);

  Serial.println("[OMX] INIT COMPLETE");

  // OpenManipulator 내부 ID 15 제어 비활성화
  omx.disableAllToolActuator();

  const char *log = nullptr;

  if (!gripper_wb.init(
        GRIPPER_DEVICE_NAME,
        GRIPPER_BAUDRATE,
        &log
      ))
  {
    Serial.print(
      "[GRIPPER] BUS INIT FAILED: "
    );

    Serial.println(
      log ? log : "unknown"
    );

    return false;
  }

  uint16_t model_number = 0;

  if (!gripper_wb.ping(
        GRIPPER_ID,
        &model_number,
        &log
      ))
  {
    Serial.print(
      "[GRIPPER] ID15 PING FAILED: "
    );

    Serial.println(
      log ? log : "unknown"
    );

    return false;
  }

  if (!gripper_wb.torqueOff(
        GRIPPER_ID,
        &log
      ))
  {
    Serial.print(
      "[GRIPPER] TORQUE OFF FAILED: "
    );

    Serial.println(
      log ? log : "unknown"
    );

    return false;
  }

  if (!gripper_wb.currentBasedPositionMode(
        GRIPPER_ID,
        GRIPPER_CURRENT,
        &log
      ))
  {
    Serial.print(
      "[GRIPPER] MODE SET FAILED: "
    );

    Serial.println(
      log ? log : "unknown"
    );

    return false;
  }

  if (!gripper_wb.itemWrite(
        GRIPPER_ID,
        "Profile_Acceleration",
        GRIPPER_PROFILE_ACCELERATION,
        &log
      ))
  {
    Serial.print(
      "[GRIPPER] ACCEL SET FAILED: "
    );

    Serial.println(
      log ? log : "unknown"
    );

    return false;
  }

  if (!gripper_wb.itemWrite(
        GRIPPER_ID,
        "Profile_Velocity",
        GRIPPER_PROFILE_VELOCITY,
        &log
      ))
  {
    Serial.print(
      "[GRIPPER] VELOCITY SET FAILED: "
    );

    Serial.println(
      log ? log : "unknown"
    );

    return false;
  }

  gripper_wb.itemWrite(
    GRIPPER_ID,
    "Return_Delay_Time",
    0,
    &log
  );

  if (!gripper_wb.torqueOn(
        GRIPPER_ID,
        &log
      ))
  {
    Serial.print(
      "[GRIPPER] TORQUE ON FAILED: "
    );

    Serial.println(
      log ? log : "unknown"
    );

    return false;
  }

  gripper_open_position =
    gripper_wb.convertRadian2Value(
      GRIPPER_ID,
      GRIPPER_OPEN_MOTOR_RAD
    );

  gripper_close_position =
    gripper_wb.convertRadian2Value(
      GRIPPER_ID,
      GRIPPER_CLOSE_MOTOR_RAD
    );

  gripper_ready = true;

  int32_t present_position = 0;

  readGripperPosition(
    present_position,
    false
  );

  Serial.print(
    "[GRIPPER] ID15 READY, MODEL="
  );

  Serial.println(
    model_number
  );

  Serial.print(
    "[GRIPPER] OPEN RAW="
  );

  Serial.println(
    gripper_open_position
  );

  Serial.print(
    "[GRIPPER] CLOSE RAW="
  );

  Serial.println(
    gripper_close_position
  );

  Serial.print(
    "[GRIPPER] PRESENT RAW="
  );

  Serial.println(
    present_position
  );

  return true;
}

// =====================================================
// J1~J4 처리
// =====================================================

void processManipulatorOnce()
{
  const double present_time =
    static_cast<double>(millis()) /
    1000.0;

  std::vector<
    robotis_manipulator::JointValue
  > goal_joint_value =
    omx.getJointGoalValueFromTrajectory(
      present_time
    );

  omx.receiveAllJointActuatorValue();

  if (!goal_joint_value.empty())
  {
    omx.sendAllJointActuatorValue(
      goal_joint_value
    );
  }

  omx.solveForwardKinematics();
}

void runManipulator(double seconds)
{
  if (seconds <= 0.0)
  {
    processManipulatorOnce();
    return;
  }

  const unsigned long start_ms =
    millis();

  const unsigned long duration_ms =
    static_cast<unsigned long>(
      seconds * 1000.0
    );

  while (
    millis() - start_ms <
    duration_ms
  )
  {
    processManipulatorOnce();
    delay(10);
  }

  processManipulatorOnce();
}

// =====================================================
// 자세 이동
// 매번 실제 현재 J1~J4 위치를 새 궤적 시작점으로 사용
// =====================================================

bool movePoseTimed(
  const JointPose &pose,
  double move_time
)
{
  if (move_time < MIN_MOVE_TIME)
  {
    move_time = MIN_MOVE_TIME;
  }

  if (move_time > MAX_MOVE_TIME)
  {
    move_time = MAX_MOVE_TIME;
  }

  std::vector<
    robotis_manipulator::JointValue
  > present_joint_value;

  if (!readCurrentJointValues(
        present_joint_value
      ))
  {
    return false;
  }

  joint_goal[0] = pose.j1;
  joint_goal[1] = pose.j2;
  joint_goal[2] = pose.j3;
  joint_goal[3] = pose.j4;

  Serial.print("[ARM] MOVE TIME=");
  Serial.println(move_time, 2);

  omx.makeJointTrajectory(
    joint_goal,
    move_time,
    present_joint_value
  );

  runManipulator(
    move_time + 0.10
  );

  return waitUntilPoseReached(
    pose
  );
}

bool movePoseAtSpeed(
  const JointPose &pose,
  double speed_rad_s
)
{
  std::vector<
    robotis_manipulator::JointValue
  > present_joint_value;

  if (!readCurrentJointValues(
        present_joint_value
      ))
  {
    return false;
  }

  const double move_time =
    calculateMoveTimeFromPresent(
      pose,
      speed_rad_s,
      present_joint_value
    );

  return movePoseTimed(
    pose,
    move_time
  );
}

// =====================================================
// 그리퍼
// =====================================================

bool openGripper()
{
  return commandGripperAndWait(
    gripper_open_position,
    "OPEN",
    false
  );
}

bool closeGripper()
{
  return commandGripperAndWait(
    gripper_close_position,
    "CLOSE",
    true
  );
}