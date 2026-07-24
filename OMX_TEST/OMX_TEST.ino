#include "utils.h"
#include "step.h"

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

static bool sequence_failed = false;

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
    sequence_failed = true;
    return;
  }

  runManipulator(1.0);

  if (!runFullSequence())
  {
    Serial.println("[SYSTEM] SEQUENCE ABORTED");
    sequence_failed = true;
  }
}

void loop()
{
  processManipulatorOnce();

  if (sequence_failed)
  {
    delay(20);
    return;
  }

  delay(10);
}