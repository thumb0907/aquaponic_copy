#include "utils.h"
#include "step.h"
#include "comm.h"

static constexpr double ARM_SPEED_RAD_S = 0.60;

static constexpr double FIRST_HOME_MOVE_TIME = 3.0;

// =====================================================
// 매니퓰레이터 자세
// =====================================================

// 홈 자세
// 수확 후 식물을 들어 올리는 자세로도 사용
static const JointPose HOME_LIFT_POSE = {
  1.1842331682475082,
  -0.5737088146694367,
  0.6550097964269619,
  -0.2807184841832795
};

// 식물을 상자에 놓은 뒤 거치는 중간 자세
// 파종 컨베이어에 트레이를 놓은 뒤에도 이 자세를 거쳐 홈 복귀
static const JointPose SAFE_TRANSFER_POSE = {
  0.019941750242306266,
  -0.3896311201231599,
  0.26231071472823775,
  0.31753402309212087
};

// 1~5번 칸 수확 자세
static const JointPose HARVEST_POSES[5] = {
  // 1번 칸
  {
    1.5677283652189180,
    0.8943107993371218,
    0.9848156658223743,
    -1.3928545554003690
  },

  // 2번 칸
  {
    1.5523885573400618,
    0.9618059540040900,
    0.8406214717611245,
    -1.5815341923103030
  },

  // 3번 칸
  {
    1.5539225381279476,
    0.8728350683067232,
    0.7225049510939301,
    -1.2271846303087200
  },

  // 4번 칸
  {
    1.5585244804916040,
    0.9173205111554061,
    0.5829126993963367,
    -1.2379224958239194
  },

  // 5번 칸
  {
    1.5968740001887456,
    0.9648739155798616,
    0.4019029664258311,
    -1.0891263593990121
  }
};

// 식물 상자에 수확물을 떨어뜨리는 자세
static const JointPose PLANT_DROP_POSE = {
  -2.8593401886190420,
  0.6151262959419350,
  0.8191457407307254,
  -0.9894176081864456
};

// 새 트레이 접근 및 들어 올림 전환 자세
static const JointPose NEW_TRAY_TRANSFER_POSE = {
  -2.9437091319527524,
  0.2853204265465221,
  0.17947575218241285,
  -0.4249126782445294
};

// 새 트레이를 집는 자세
static const JointPose NEW_TRAY_PICKUP_POSE = {
  -3.0250101137106915,
  1.2394564766113910,
  -0.23930100291036682,
  -0.5737088146694367
};

// 파종 컨베이어에 트레이를 내려놓는 자세
static const JointPose SEEDING_PLACE_POSE = {
  -1.6873788666744123,
  0.8973787609128934,
  0.6151262959419350,
  -1.6536312893409282
};

// 통신만 먼저 확인할 때 true
// 실제 매니퓰레이터 동작까지 실행할 때 false
static constexpr bool COMM_TEST_ONLY = false;

static bool sequence_failed = false;
static bool manipulator_ready = false;
static bool estopped = false;
static bool job_running = false;

static uint8_t active_job_id = 0;
static uint8_t last_completed_job_id = 0;
static bool last_completed_valid = false;
static void enterEstop()
{
  const uint8_t stopped_job_id =
    active_job_id;

  stopRail();

  estopped = true;
  sequence_failed = true;
  job_running = false;

  commSendState(
    MANIP_STATE_ESTOP,
    stopped_job_id
  );

  active_job_id = 0;
}

// =====================================================
// 이동 결과 확인
// =====================================================

static bool checkMove(
  bool result,
  const char *move_name
)
{
  if (result)
  {
    return true;
  }

  Serial.print("[SYSTEM] MOVE FAILED: ");
  Serial.println(move_name);

  return false;
}

// =====================================================
// 홈 자세 이동
// =====================================================

static bool moveHome()
{
  Serial.println("[ARM] MOVE HOME");

  return checkMove(
    movePoseAtSpeed(
      HOME_LIFT_POSE,
      ARM_SPEED_RAD_S
    ),
    "HOME"
  );
}

// =====================================================
// 식물 놓기용 그리퍼 반복 동작
// =====================================================
//
// 식물 상자 자세 도착 후:
//
// 열기
// → 닫기
// → 다시 열기  (1회)
// → 닫기
// → 다시 열기  (2회)
//
// J2/J4 흔들기는 사용하지 않음.
// =====================================================

static bool releasePlantWithGripperMotion()
{
  Serial.println("[HARVEST] RELEASE OPEN");

  if (!openGripper())
  {
    Serial.println(
      "[SYSTEM] PLANT RELEASE OPEN FAILED"
    );

    return false;
  }

  for (uint8_t repeat = 0; repeat < 2; ++repeat)
  {
    Serial.print("[HARVEST] RELEASE CLOSE/OPEN ");
    Serial.println(repeat + 1);

    if (!closeGripper())
    {
      Serial.println(
        "[SYSTEM] PLANT RELEASE CLOSE FAILED"
      );

      return false;
    }

    if (!openGripper())
    {
      Serial.println(
        "[SYSTEM] PLANT RELEASE REOPEN FAILED"
      );

      return false;
    }
  }

  return true;
}

// =====================================================
// 트레이 놓기용 그리퍼 반복 동작
// =====================================================
//
// 파종 컨베이어 자세 도착 후:
//
// 열기
// → 닫기
// → 다시 열기  (1회)
// → 닫기
// → 다시 열기  (2회)
//
// J2/J4 흔들기는 사용하지 않음.
// =====================================================

static bool releaseTrayWithGripperMotion()
{
  Serial.println("[NEW TRAY] RELEASE OPEN");

  if (!openGripper())
  {
    Serial.println(
      "[SYSTEM] TRAY RELEASE OPEN FAILED"
    );

    return false;
  }

  for (uint8_t repeat = 0; repeat < 2; ++repeat)
  {
    Serial.print("[NEW TRAY] RELEASE CLOSE/OPEN ");
    Serial.println(repeat + 1);

    if (!closeGripper())
    {
      Serial.println(
        "[SYSTEM] TRAY RELEASE CLOSE FAILED"
      );

      return false;
    }

    if (!openGripper())
    {
      Serial.println(
        "[SYSTEM] TRAY RELEASE REOPEN FAILED"
      );

      return false;
    }
  }

  return true;
}

// =====================================================
// 한 칸 수확
// =====================================================
//
// 순서:
// 해당 칸 수확 자세
// → 그리퍼 닫기
// → 들어 올림 자세
// → 중간 자세
// → 식물 상자 자세
// → 그리퍼 열기
// → 닫기/열기 2회
// → 중간 자세
// → 홈 복귀
//
// 1번부터 5번까지 모두 같은 순서로 실행함.
// =====================================================

static bool harvestSlot(uint8_t slot)
{
  Serial.print("[HARVEST] SLOT ");
  Serial.print(slot + 1);
  Serial.println(" START");

  // 1. 해당 칸 수확 자세
  Serial.println("[HARVEST] MOVE TO HARVEST POSITION");

  if (!checkMove(
        movePoseAtSpeed(
          HARVEST_POSES[slot],
          ARM_SPEED_RAD_S
        ),
        "HARVEST POSITION"
      ))
  {
    return false;
  }

  // 2. 식물 집기
  Serial.println("[HARVEST] GRIP");

  if (!closeGripper())
  {
    Serial.println(
      "[SYSTEM] GRIPPER CLOSE FAILED"
    );

    return false;
  }

  // 3. 식물을 잡은 상태로 들어 올림
  Serial.println("[HARVEST] LIFT");

  if (!checkMove(
        movePoseAtSpeed(
          HOME_LIFT_POSE,
          ARM_SPEED_RAD_S
        ),
        "HARVEST LIFT"
      ))
  {
    return false;
  }

  // 4. 식물 상자로 이동하기 전 중간 자세
  Serial.println("[HARVEST] SAFE TRANSFER BEFORE DROP");

  if (!checkMove(
        movePoseAtSpeed(
          SAFE_TRANSFER_POSE,
          ARM_SPEED_RAD_S
        ),
        "SAFE TRANSFER BEFORE DROP"
      ))
  {
    return false;
  }

  // 5. 식물 상자로 이동
  Serial.println("[HARVEST] MOVE TO PLANT BOX");

  if (!checkMove(
        movePoseAtSpeed(
          PLANT_DROP_POSE,
          ARM_SPEED_RAD_S
        ),
        "PLANT DROP POSITION"
      ))
  {
    return false;
  }

  // 6. 식물 놓기: 열기 후 닫기/열기 2회
  if (!releasePlantWithGripperMotion())
  {
    return false;
  }

  // 7. 식물을 놓은 뒤 다시 중간 자세
  Serial.println("[HARVEST] SAFE TRANSFER AFTER DROP");

  if (!checkMove(
        movePoseAtSpeed(
          SAFE_TRANSFER_POSE,
          ARM_SPEED_RAD_S
        ),
        "SAFE TRANSFER AFTER DROP"
      ))
  {
    return false;
  }

  // 8. 모든 칸에서 홈 복귀
  Serial.println("[HARVEST] RETURN HOME");

  if (!moveHome())
  {
    return false;
  }

  Serial.print("[HARVEST] SLOT ");
  Serial.print(slot + 1);
  Serial.println(" COMPLETE");

  return true;
}

// =====================================================
// 새 트레이 이송
// =====================================================
//
// 5번 칸도 홈 복귀한 뒤 새 트레이 공정을 시작함.
//
// 순서:
// 전환 자세
// → 새 트레이 집는 자세
// → 그리퍼 닫기
// → 전환 자세
// → 파종 컨베이어 자세
// → 그리퍼 열기
// → 닫기/열기 2회
// → 중간 자세
// → 홈 복귀
// =====================================================

static bool transferNewTray()
{
  Serial.println("[NEW TRAY] START");

  // 1. 새 트레이 접근용 전환 자세
  Serial.println("[NEW TRAY] MOVE TO TRANSFER POSE");

  if (!checkMove(
        movePoseAtSpeed(
          NEW_TRAY_TRANSFER_POSE,
          ARM_SPEED_RAD_S
        ),
        "NEW TRAY TRANSFER BEFORE PICKUP"
      ))
  {
    return false;
  }

  // 2. 새 트레이 집는 자세
  Serial.println("[NEW TRAY] MOVE TO PICKUP POSITION");

  if (!checkMove(
        movePoseAtSpeed(
          NEW_TRAY_PICKUP_POSE,
          ARM_SPEED_RAD_S
        ),
        "NEW TRAY PICKUP POSITION"
      ))
  {
    return false;
  }

  // 3. 새 트레이 집기
  Serial.println("[NEW TRAY] GRIP");

  if (!closeGripper())
  {
    Serial.println(
      "[SYSTEM] NEW TRAY GRIP FAILED"
    );

    return false;
  }

  // 4. 새 트레이를 잡은 상태로 전환 자세
  Serial.println("[NEW TRAY] LIFT TO TRANSFER POSE");

  if (!checkMove(
        movePoseAtSpeed(
          NEW_TRAY_TRANSFER_POSE,
          ARM_SPEED_RAD_S
        ),
        "NEW TRAY TRANSFER AFTER PICKUP"
      ))
  {
    return false;
  }

  // 5. 파종 컨베이어로 이동
  Serial.println("[NEW TRAY] MOVE TO SEEDING CONVEYOR");

  if (!checkMove(
        movePoseAtSpeed(
          SEEDING_PLACE_POSE,
          ARM_SPEED_RAD_S
        ),
        "SEEDING PLACE POSITION"
      ))
  {
    return false;
  }

  // 6. 파종 컨베이어에 트레이 놓기
  // 식물과 동일하게 열기 후 닫기/열기 동작을 2회 수행
  if (!releaseTrayWithGripperMotion())
  {
    return false;
  }

  // 7. 중간 자세
  Serial.println("[NEW TRAY] SAFE TRANSFER POSE");

  if (!checkMove(
        movePoseAtSpeed(
          SAFE_TRANSFER_POSE,
          ARM_SPEED_RAD_S
        ),
        "SAFE TRANSFER AFTER TRAY RELEASE"
      ))
  {
    return false;
  }

  // 9. 최종 홈 복귀
  Serial.println("[NEW TRAY] RETURN HOME");

  if (!moveHome())
  {
    return false;
  }

  Serial.println("[NEW TRAY] COMPLETE");

  return true;
}

// =====================================================
// 전체 시퀀스
// =====================================================
//
// 1. 매니퓰레이터 홈
// 2. 그리퍼 열기
// 3. 리니어 레일 호밍
//
// 4. 1번 수확
//    → 들어 올림
//    → 중간 자세
//    → 식물 상자
//    → 열기
//    → 닫기/열기 2회
//    → 중간 자세
//    → 홈
//
// 5~8. 2~5번도 동일
//
// 9. 새 트레이 공정
//    → 전환 자세
//    → 새 트레이 집기
//    → 전환 자세
//    → 파종 컨베이어에 놓기
//    → 닫기/열기 2회
//    → 중간 자세
//    → 홈
// =====================================================

static bool runFullSequence()
{
  // 1. 매니퓰레이터 홈
  Serial.println("[SEQUENCE] MANIPULATOR HOME");

  if (!checkMove(
        movePoseTimed(
          HOME_LIFT_POSE,
          FIRST_HOME_MOVE_TIME
        ),
        "INITIAL HOME"
      ))
  {
    return false;
  }

  // 2. 그리퍼 열기
  Serial.println("[SEQUENCE] OPEN GRIPPER");

  if (!openGripper())
  {
    Serial.println(
      "[SYSTEM] INITIAL GRIPPER OPEN FAILED"
    );

    return false;
  }

  // 3. 리니어 레일 호밍
  Serial.println("[SEQUENCE] RAIL HOMING");

  if (!homeRail())
  {
    Serial.println(
      "[SYSTEM] RAIL HOMING FAILED"
    );

    return false;
  }

  // 레일 호밍 중 멈췄던 매니퓰레이터 처리 재개
  if (!runManipulator(0.5))
  {
    return false;
  }

  // 4~8. 1번부터 5번까지 순차 수확
  for (uint8_t slot = 0; slot < 5; ++slot)
  {
    if (!harvestSlot(slot))
    {
      return false;
    }
  }

  // 9. 새 트레이 이송
  if (!transferNewTray())
  {
    return false;
  }

  Serial.println(
    "[SYSTEM] FULL SEQUENCE COMPLETE"
  );

  return true;
}

// =====================================================
// Arduino 기본 함수
// =====================================================

void setup()
{
  Serial.begin(115200);
  commBegin(Serial);

  const unsigned long serial_start_ms =
    millis();

  while (
    !Serial &&
    millis() - serial_start_ms < 3000
  )
  {
    delay(10);
  }

  delay(300);

  initializeRail();

  if (!initManipulator())
  {
    manipulator_ready = false;
    sequence_failed = true;

    commSendState(
      MANIP_STATE_ERROR,
      0
    );

    commSendError(
      0,
      MANIP_ERR_NOT_READY
    );

    return;
  }

  // Dynamixel/OpenManipulator 초기 처리
  runManipulator(1.0);

  manipulator_ready = true;
  sequence_failed = false;
  estopped = false;
  job_running = false;
  active_job_id = 0;

  // Pi2에 준비 완료 보고
  commSendState(
    MANIP_STATE_IDLE,
    0
  );
}

void loop()
{
  processManipulatorOnce();
  commPoll();

  // ESTOP은 RESET보다 먼저 처리
  if (commTakeEstop())
  {
    enterEstop();
    return;
  }

  if (commTakeReset())
  {
    stopRail();

    estopped = false;
    sequence_failed = false;
    job_running = false;
    active_job_id = 0;

    // RESET 이후 같은 ID 작업을 새 작업으로 받을 수 있게 초기화
    last_completed_valid = false;

    if (manipulator_ready)
    {
      commSendState(
        MANIP_STATE_IDLE,
        0
      );
    }
    else
    {
      commSendState(
        MANIP_STATE_ERROR,
        0
      );
    }

    return;
  }

  uint8_t requested_job_id = 0;

  if (!commTakeHarvestJob(requested_job_id))
  {
    delay(10);
    return;
  }

  // 이미 완료한 작업을 Master가 다시 보낸 경우
  // 실제 동작은 반복하지 않고 DONE만 재전송
  if (
    last_completed_valid &&
    requested_job_id == last_completed_job_id
  )
  {
    commSendDone(requested_job_id);
    return;
  }

  if (!manipulator_ready)
  {
    commSendError(
      requested_job_id,
      MANIP_ERR_NOT_READY
    );
    return;
  }

  if (estopped)
  {
    commSendError(
      requested_job_id,
      MANIP_ERR_ESTOP
    );
    return;
  }

  // 이전 동작 실패 후 RESET하지 않은 상태
  if (sequence_failed)
  {
    commSendError(
      requested_job_id,
      MANIP_ERR_SEQUENCE_FAILED
    );
    return;
  }

  if (job_running)
  {
    commSendError(
      requested_job_id,
      MANIP_ERR_BUSY
    );
    return;
  }

  job_running = true;
  active_job_id = requested_job_id;

  commSendState(
    MANIP_STATE_HARVESTING,
    active_job_id
  );

  bool result = false;

  if (COMM_TEST_ONLY)
  {
    // 통신 검증용 가상 수확
    result = runManipulator(3.0);
    result = true;
  }
  else
  {
    // 최신 manip 폴더의 실제 동작
    result = runFullSequence();
  }

  // 동작 도중 들어온 ESTOP이 있었는지 한 번 더 확인
  commPoll();

  if (commTakeEstop())
  {
    enterEstop();
    return;
  }

  const uint8_t finished_job_id =
    active_job_id;

  if (result)
  {
    last_completed_job_id =
      finished_job_id;
    last_completed_valid = true;

    sequence_failed = false;

    commSendDone(
      finished_job_id
    );
  }
  else
  {
    sequence_failed = true;

    commSendError(
      finished_job_id,
      MANIP_ERR_SEQUENCE_FAILED
    );

    commSendState(
      MANIP_STATE_ERROR,
      finished_job_id
    );
  }

  job_running = false;
  active_job_id = 0;

  if (!sequence_failed)
  {
    commSendState(
      MANIP_STATE_IDLE,
      0
    );
  }

  delay(10);
}