#include "step.h"
#include "comm.h"
#include <math.h>

static constexpr int SW_PIN = 3;
static constexpr int PUL_PIN = 4;
static constexpr int DIR_PIN = 7;

static constexpr bool RAIL_HOME_DIR = 1;
static constexpr bool RAIL_AWAY_DIR = 0;

static constexpr float PULSES_PER_MM = 500.0f;

static constexpr int RAIL_HOME_DELAY_US = 500;
static constexpr int RAIL_START_DELAY_US = 1000;
static constexpr int RAIL_MOVE_DELAY_US = 180;

static constexpr float RAIL_BACKOFF_MM = 3.0f;

static constexpr unsigned long RAIL_HOME_TIMEOUT_MS =
  30000;

static long mmToPulses(float mm)
{
  return lround(
    fabs(mm) *
    PULSES_PER_MM
  );
}

void initializeRail()
{
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT_PULLUP);

  digitalWrite(PUL_PIN, HIGH);
  digitalWrite(DIR_PIN, LOW);

  Serial.println("[RAIL] INITIALIZED");
}

void stopRail()
{
  digitalWrite(PUL_PIN, HIGH);
}

static void stepOnce(int delay_us)
{
  digitalWrite(PUL_PIN, LOW);
  delayMicroseconds(20);

  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(delay_us);
}

static int calculateRampDelay(
  long current_pulse,
  long total_pulses,
  long ramp_pulses,
  int start_delay,
  int target_delay
)
{
  if (ramp_pulses <= 0)
  {
    return target_delay;
  }

  if (current_pulse < ramp_pulses)
  {
    return start_delay -
      static_cast<long>(
        start_delay - target_delay
      ) *
      current_pulse /
      ramp_pulses;
  }

  const long remaining =
    total_pulses - current_pulse;

  if (remaining < ramp_pulses)
  {
    return start_delay -
      static_cast<long>(
        start_delay - target_delay
      ) *
      remaining /
      ramp_pulses;
  }

  return target_delay;
}

static bool moveRailMM(
  bool direction,
  float distance_mm,
  int target_delay_us
)
{
  const long pulses =
    mmToPulses(distance_mm);

  long ramp_pulses = 750;

  if (pulses < ramp_pulses * 2)
  {
    ramp_pulses =
      pulses / 2;
  }

  digitalWrite(
    DIR_PIN,
    direction ? HIGH : LOW
  );

  delayMicroseconds(100);
  commPoll();

  if (commEstopPending())
  {
    stopRail();
    return false;
  }
  for (long i = 0; i < pulses; ++i)
  {
    const int pulse_delay =
      calculateRampDelay(
        i,
        pulses,
        ramp_pulses,
        RAIL_START_DELAY_US,
        target_delay_us
      );

    stepOnce(pulse_delay);
  }

  stopRail();
  stopRail();
  return true;  
}

bool homeRail()
{
  Serial.println("[RAIL] HOMING START");

  if (digitalRead(SW_PIN) == LOW)
  {
    if (!moveRailMM(
          RAIL_AWAY_DIR,
          RAIL_BACKOFF_MM,
          RAIL_MOVE_DELAY_US
        ))
    {
      return false;
    }
    );

    delay(300);

    if (digitalRead(SW_PIN) == LOW)
    {
      Serial.println(
        "[RAIL] LIMIT RELEASE FAILED"
      );

      return false;
    }
  }

  digitalWrite(
    DIR_PIN,
    RAIL_HOME_DIR ? HIGH : LOW
  );

  delayMicroseconds(100);

  const unsigned long start_ms =
    millis();

  while (digitalRead(SW_PIN) == HIGH)
  {
    commPoll();

    if (commEstopPending())
    {
      stopRail();
      return false;
    }
    if (
      millis() - start_ms >
      RAIL_HOME_TIMEOUT_MS
    )
    {
      stopRail();

      Serial.println(
        "[RAIL] HOMING TIMEOUT"
      );

      return false;
    }

    stepOnce(
      RAIL_HOME_DELAY_US
    );
  }

  stopRail();

  Serial.println("[RAIL] LIMIT DETECTED");

  delay(300);

  if (!moveRailMM(
          RAIL_AWAY_DIR,
          RAIL_BACKOFF_MM,
          RAIL_MOVE_DELAY_US
        ))
    {
      return false;
    }

  delay(300);

  if (digitalRead(SW_PIN) == LOW)
  {
    Serial.println(
      "[RAIL] BACKOFF FAILED"
    );

    return false;
  }

  Serial.println(
    "[RAIL] HOMING COMPLETE"
  );

  return true;
}