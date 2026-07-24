#include "utils.h"
#include "step.h"
#include "comm.h"

static constexpr double ARM_SPEED_RAD_S = 0.42;
static constexpr double FIRST_HOME_TIME = 4.0;
static constexpr double POSE_SETTLE_TIME = 0.5;
static constexpr double NEW_TRAY_WAIT_TIME = 5.0;

static const JointPose HOME_LIFT_POSE = {
  1.1842331682475082,
  -0.5737088146694367,
  0.6550097964269619,
  -0.2807184841832795
};

static const JointPose HARVEST_POSES[5] = {
  {
    1.4695535947942373,
    1.1934370529748222,
    0.9464661461252337,
    -1.8745245227964604
  },
  {
    1.5431846726127478,
    1.1228739367320824,
    0.8298836062459247,
    -1.8714565612206893
  },
  {
    1.5431846726127478,
    1.1750292835201943,
    0.6764855274573609,
    -1.8714565612206893
  },
  {
    1.5585244804916040,
    1.0952622825501406,
    0.6089903727903927,
    -1.7824856755233220
  },
  {
    1.5539225381279476,
    1.1719613219444227,
    0.34361169648617684,
    -1.6076118657043590
  }
};

static const JointPose PLANT_DROP_POSE = {
  -2.8593401886190420,
  0.6151262959419350,
  0.8191457407307254,
  -0.9894176081864456
};

static const JointPose NEW_TRAY_PICKUP_POSE = {
  -3.1001751723170880,
  1.4986992297640649,
  -0.7900001057613122,
  -0.8022719520643973
};

static const JointPose SEEDING_PLACE_POSE = {
  -1.6873788666744123,
  0.8973787609128934,
  0.6151262959419350,
  -1.6536312893409282
};

// 처음에는 실제 모터를 움직이지 않고 통신만 시험
static constexpr bool COMM_TEST_ONLY = true;

static bool sequence_failed = false;
static bool manipulator_ready = false;
static bool estopped = false;
static bool job_running = false;

static uint8_t active_job_id = 0;
static uint8_t last_completed_job_id = 0;
static bool last_completed_valid = false;

static void moveHomeOpen()
{
  movePoseAtSpeed(HOME_LIFT_POSE, ARM_SPEED_RAD_S);
  runManipulator(POSE_SETTLE_TIME);
}

static bool harvestSlot(uint8_t slot)
{
  Serial.print("[HARVEST] SLOT ");
  Serial.println(slot + 1);

  movePoseAtSpeed(HARVEST_POSES[slot], ARM_SPEED_RAD_S);
  runManipulator(POSE_SETTLE_TIME);

  if (!closeGripper())
  {
    Serial.println("[SYSTEM] GRIPPER CLOSE FAILED");
    return false;
  }

  movePoseAtSpeed(HOME_LIFT_POSE, ARM_SPEED_RAD_S);
  runManipulator(POSE_SETTLE_TIME);

  movePoseAtSpeed(PLANT_DROP_POSE, ARM_SPEED_RAD_S);
  runManipulator(POSE_SETTLE_TIME);

  if (!openGripper())
  {
    Serial.println("[SYSTEM] GRIPPER OPEN FAILED");
    return false;
  }

  if (slot < 4)
  {
    moveHomeOpen();
  }

  return true;
}

static bool transferNewTray()
{
  Serial.println("[NEW TRAY] WAIT 5 SEC");
  runManipulator(NEW_TRAY_WAIT_TIME);

  Serial.println("[NEW TRAY] PICKUP");
  movePoseAtSpeed(NEW_TRAY_PICKUP_POSE, ARM_SPEED_RAD_S);
  runManipulator(POSE_SETTLE_TIME);

  if (!closeGripper())
  {
    Serial.println("[SYSTEM] TRAY GRIP FAILED");
    return false;
  }

  Serial.println("[NEW TRAY] SEEDING PLACE");
  movePoseAtSpeed(SEEDING_PLACE_POSE, ARM_SPEED_RAD_S);
  runManipulator(POSE_SETTLE_TIME);

  if (!openGripper())
  {
    Serial.println("[SYSTEM] TRAY RELEASE FAILED");
    return false;
  }

  moveHomeOpen();
  Serial.println("[NEW TRAY] COMPLETE");
  return true;
}

static bool runFullSequence()
{
  Serial.println("[SEQUENCE] HOME");
  movePoseTimed(HOME_LIFT_POSE, FIRST_HOME_TIME);

  if (!openGripper())
  {
    Serial.println("[SYSTEM] INITIAL GRIPPER OPEN FAILED");
    return false;
  }

  runManipulator(2.0);

  Serial.println("[SEQUENCE] RAIL HOME");
  if (!homeRail())
  {
    Serial.println("[SYSTEM] RAIL HOMING FAILED");
    return false;
  }

  runManipulator(0.5);

  for (uint8_t slot = 0; slot < 5; ++slot)
  {
    if (!harvestSlot(slot))
    {
      return false;
    }
  }

  if (!transferNewTray())
  {
    return false;
  }

  Serial.println("[SYSTEM] FULL SEQUENCE COMPLETE");
  return true;
}

void setup()
{
  Serial.begin(115200);
  commBegin(Serial);

  const unsigned long start_ms = millis();

  while (!Serial && millis() - start_ms < 3000)
  {
    delay(10);
  }

  delay(300);

  initializeRail();

  if (!initManipulator())
  {
    Serial.println("[SYSTEM] INIT FAILED");

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

  runManipulator(1.0);

  manipulator_ready = true;
  sequence_failed = false;
  estopped = false;

  Serial.println("[SYSTEM] WAITING FOR PI2 COMMAND");

  commSendState(
    MANIP_STATE_IDLE,
    0
  );
}

void loop()
{
  processManipulatorOnce();
  commPoll();

  // ─────────────────────────────
  // RESET 처리
  // ─────────────────────────────
  if (commTakeReset())
  {
    stopRail();

    estopped = false;
    sequence_failed = false;
    job_running = false;
    active_job_id = 0;

    last_completed_valid = false;

    Serial.println("[COMM] RESET");

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

  // ─────────────────────────────
  // ESTOP 처리
  // ─────────────────────────────
  if (commTakeEstop())
  {
    stopRail();

    estopped = true;
    sequence_failed = true;
    job_running = false;

    Serial.println("[COMM] ESTOP");

    commSendState(
      MANIP_STATE_ESTOP,
      active_job_id
    );

    return;
  }

  // ─────────────────────────────
  // 새 수확 작업 확인
  // ─────────────────────────────
  uint8_t requested_job_id = 0;

  if (!commTakeHarvestJob(requested_job_id))
  {
    delay(10);
    return;
  }

  Serial.print("[COMM] HARVEST JOB RECEIVED, ID=");
  Serial.println(requested_job_id);

  // 같은 완료 패킷이 재전송된 경우
  // 실제 수확을 다시 하지 않고 DONE만 다시 응답
  if (
    last_completed_valid &&
    requested_job_id == last_completed_job_id
  )
  {
    Serial.println("[COMM] DUPLICATE COMPLETED JOB");

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

  if (job_running)
  {
    commSendError(
      requested_job_id,
      MANIP_ERR_BUSY
    );

    return;
  }

  // ─────────────────────────────
  // 수확 작업 시작
  // ─────────────────────────────
  job_running = true;
  active_job_id = requested_job_id;

  commSendState(
    MANIP_STATE_HARVESTING,
    active_job_id
  );

  bool result = false;

  if (COMM_TEST_ONLY)
  {
    Serial.println("[COMM TEST] 3 SECOND FAKE HARVEST");

    runManipulator(3.0);
    result = true;
  }
  else
  {
    result = runFullSequence();
  }

  // ─────────────────────────────
  // 결과 보고
  // ─────────────────────────────
  if (result)
  {
    Serial.println("[COMM] HARVEST COMPLETE");

    last_completed_job_id = active_job_id;
    last_completed_valid = true;

    // 반드시 ID를 지우기 전에 DONE 전송
    commSendDone(active_job_id);

    sequence_failed = false;
  }
  else
  {
    Serial.println("[COMM] HARVEST FAILED");

    commSendError(
      active_job_id,
      MANIP_ERR_SEQUENCE_FAILED
    );

    sequence_failed = true;
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