#include "utils.h"

#include <math.h>
#include <vector>

OpenManipulator omx;
DynamixelWorkbench gripper_wb;

// =====================================================
// 그리퍼 ID 15 설정
// =====================================================

static constexpr char GRIPPER_DEVICE_NAME[] = "";
static constexpr uint32_t GRIPPER_BAUDRATE = 1000000;
static constexpr uint8_t GRIPPER_ID = 15;

// 현재 설치 방향 기준
// 열림이 부족하면 OPEN 값을 0.1씩 증가
static constexpr float GRIPPER_OPEN_MOTOR_RAD = 1.20f;
static constexpr float GRIPPER_CLOSE_MOTOR_RAD = -0.6666667f;

static constexpr int32_t GRIPPER_CURRENT = 200;
static constexpr int32_t GRIPPER_PROFILE_ACCELERATION = 20;
static constexpr int32_t GRIPPER_PROFILE_VELOCITY = 200;

// 목표 위치 판정 설정
static constexpr int32_t GRIPPER_POSITION_TOLERANCE = 20;

// 식물을 잡아서 목표 위치까지 못 가더라도
// 이 값 이상 닫혔으면 파지한 것으로 판단
static constexpr int32_t GRIPPER_MIN_CLOSE_TRAVEL = 80;

static constexpr unsigned long GRIPPER_TIMEOUT_MS = 4000;
static constexpr unsigned long GRIPPER_COMMAND_REPEAT_MS = 200;
static constexpr unsigned long GRIPPER_PRINT_INTERVAL_MS = 250;

// =====================================================
// 매니퓰레이터 이동 설정
// =====================================================

static constexpr double DEFAULT_FIRST_MOVE_TIME = 4.0;
static constexpr double MIN_MOVE_TIME = 1.2;
static constexpr double MAX_MOVE_TIME = 15.0;

static JointPose last_target = {
  0.0,
  0.0,
  0.0,
  0.0
};

static bool last_target_valid = false;
static bool gripper_ready = false;

static int32_t gripper_open_position = 0;
static int32_t gripper_close_position = 0;

static std::vector<double> joint_goal(4, 0.0);

// =====================================================
// 그리퍼 현재 위치 읽기
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
    Serial.print("[GRIPPER] POSITION READ FAILED: ");
    Serial.println(log ? log : "unknown");
  }

  return result;
}

// =====================================================
// 그리퍼 목표 위치 전송
// =====================================================

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
    Serial.print("[GRIPPER] GOAL WRITE FAILED: ");
    Serial.println(log ? log : "unknown");
  }

  return result;
}

// =====================================================
// 매니퓰레이터 이동시간 계산
// =====================================================

static double calculateMoveTime(
  const JointPose &target,
  double speed_rad_s
)
{
  if (!last_target_valid)
  {
    return DEFAULT_FIRST_MOVE_TIME;
  }

  if (speed_rad_s <= 0.0)
  {
    speed_rad_s = 0.42;
  }

  const double diff_j1 =
    fabs(target.j1 - last_target.j1);

  const double diff_j2 =
    fabs(target.j2 - last_target.j2);

  const double diff_j3 =
    fabs(target.j3 - last_target.j3);

  const double diff_j4 =
    fabs(target.j4 - last_target.j4);

  double max_delta = diff_j1;

  if (diff_j2 > max_delta)
  {
    max_delta = diff_j2;
  }

  if (diff_j3 > max_delta)
  {
    max_delta = diff_j3;
  }

  if (diff_j4 > max_delta)
  {
    max_delta = diff_j4;
  }

  double move_time =
    max_delta / speed_rad_s;

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
// 그리퍼 명령 후 위치 확인
// =====================================================

static bool commandGripperAndWait(
  int32_t target_position,
  const char *command_name,
  bool allow_contact_stop
)
{
  int32_t start_position = 0;

  if (!readGripperPosition(start_position, true))
  {
    return false;
  }

  Serial.print("[GRIPPER] ");
  Serial.print(command_name);
  Serial.print(" START=");
  Serial.print(start_position);
  Serial.print(" TARGET=");
  Serial.println(target_position);

  if (!writeGripperPosition(target_position, true))
  {
    return false;
  }

  const int32_t start_error =
    labs(target_position - start_position);

  const unsigned long start_ms = millis();

  unsigned long last_command_ms = 0;
  unsigned long last_print_ms = 0;

  int32_t present_position = start_position;

  while (
    millis() - start_ms <
    GRIPPER_TIMEOUT_MS
  )
  {
    processManipulatorOnce();

    const unsigned long now_ms = millis();

    // 일정 시간마다 목표 위치 다시 전송
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

    const int32_t error =
      labs(
        target_position -
        present_position
      );

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

    delay(20);
  }

  const int32_t final_error =
    labs(
      target_position -
      present_position
    );

  const int32_t moved_toward_target =
    start_error - final_error;

  // 닫는 동작은 물체를 잡으면 목표값까지
  // 도달하지 못할 수 있으므로 이동량으로 판단
  if (
    allow_contact_stop &&
    moved_toward_target >=
    GRIPPER_MIN_CLOSE_TRAVEL
  )
  {
    Serial.print("[GRIPPER] ");
    Serial.print(command_name);
    Serial.print(" CONTACT HOLD, PRESENT=");
    Serial.println(present_position);

    // 현재 목표값을 유지
    writeGripperPosition(
      target_position,
      false
    );

    return true;
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
// OpenManipulator 및 그리퍼 초기화
// =====================================================

bool initManipulator()
{
  Serial.println("[OMX] INIT START");

  omx.initOpenManipulator(true);

  Serial.println("[OMX] INIT COMPLETE");

  // OpenManipulator 내부 그리퍼 제어 비활성화
  // J1~J4는 유지하고 ID 15만 별도 제어
  omx.disableAllToolActuator();

  const char *log = nullptr;

  if (!gripper_wb.init(
        GRIPPER_DEVICE_NAME,
        GRIPPER_BAUDRATE,
        &log
      ))
  {
    Serial.print("[GRIPPER] BUS INIT FAILED: ");
    Serial.println(log ? log : "unknown");

    return false;
  }

  uint16_t model_number = 0;

  if (!gripper_wb.ping(
        GRIPPER_ID,
        &model_number,
        &log
      ))
  {
    Serial.print("[GRIPPER] ID15 PING FAILED: ");
    Serial.println(log ? log : "unknown");

    return false;
  }

  if (!gripper_wb.torqueOff(
        GRIPPER_ID,
        &log
      ))
  {
    Serial.print("[GRIPPER] TORQUE OFF FAILED: ");
    Serial.println(log ? log : "unknown");

    return false;
  }

  // Current-based Position Control Mode
  if (!gripper_wb.currentBasedPositionMode(
        GRIPPER_ID,
        GRIPPER_CURRENT,
        &log
      ))
  {
    Serial.print("[GRIPPER] MODE SET FAILED: ");
    Serial.println(log ? log : "unknown");

    return false;
  }

  if (!gripper_wb.itemWrite(
        GRIPPER_ID,
        "Profile_Acceleration",
        GRIPPER_PROFILE_ACCELERATION,
        &log
      ))
  {
    Serial.print("[GRIPPER] ACCEL SET FAILED: ");
    Serial.println(log ? log : "unknown");

    return false;
  }

  if (!gripper_wb.itemWrite(
        GRIPPER_ID,
        "Profile_Velocity",
        GRIPPER_PROFILE_VELOCITY,
        &log
      ))
  {
    Serial.print("[GRIPPER] VELOCITY SET FAILED: ");
    Serial.println(log ? log : "unknown");

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
    Serial.print("[GRIPPER] TORQUE ON FAILED: ");
    Serial.println(log ? log : "unknown");

    return false;
  }

  // 라디안 목표값을 ID 15 RAW 값으로 변환
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
  last_target_valid = false;

  int32_t present_position = 0;

  readGripperPosition(
    present_position,
    false
  );

  Serial.print("[GRIPPER] ID15 READY, MODEL=");
  Serial.println(model_number);

  Serial.print("[GRIPPER] OPEN RAD=");
  Serial.println(
    GRIPPER_OPEN_MOTOR_RAD,
    4
  );

  Serial.print("[GRIPPER] CLOSE RAD=");
  Serial.println(
    GRIPPER_CLOSE_MOTOR_RAD,
    4
  );

  Serial.print("[GRIPPER] OPEN RAW=");
  Serial.println(
    gripper_open_position
  );

  Serial.print("[GRIPPER] CLOSE RAW=");
  Serial.println(
    gripper_close_position
  );

  Serial.print("[GRIPPER] PRESENT RAW=");
  Serial.println(
    present_position
  );

  return true;
}

// =====================================================
// J1~J4만 OpenManipulator로 처리
// ID 15 그리퍼는 여기서 처리하지 않음
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

// =====================================================
// 일정 시간 매니퓰레이터 처리
// =====================================================

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
}

// =====================================================
// 지정 시간으로 관절 이동
// =====================================================

void movePoseTimed(
  const JointPose &pose,
  double move_time
)
{
  joint_goal[0] = pose.j1;
  joint_goal[1] = pose.j2;
  joint_goal[2] = pose.j3;
  joint_goal[3] = pose.j4;

  omx.makeJointTrajectory(
    joint_goal,
    move_time
  );

  runManipulator(
    move_time + 0.4
  );

  last_target = pose;
  last_target_valid = true;
}

// =====================================================
// 지정 속도로 관절 이동
// =====================================================

void movePoseAtSpeed(
  const JointPose &pose,
  double speed_rad_s
)
{
  const double move_time =
    calculateMoveTime(
      pose,
      speed_rad_s
    );

  movePoseTimed(
    pose,
    move_time
  );
}

// =====================================================
// 그리퍼 완전 열기
// =====================================================

bool openGripper()
{
  return commandGripperAndWait(
    gripper_open_position,
    "OPEN",
    false
  );
}

// =====================================================
// 그리퍼 닫기
// =====================================================

bool closeGripper()
{
  return commandGripperAndWait(
    gripper_close_position,
    "CLOSE",
    true
  );
}