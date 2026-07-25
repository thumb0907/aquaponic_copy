const int IR_SENSOR_PIN = 7;
const int MOTOR_PIN = 8;

// IR 센서: 물체 감지 시 LOW
const int IR_DETECTED_LEVEL = LOW;

// 현재 모터 제어 논리
const int MOTOR_STOP_LEVEL = HIGH;
const int MOTOR_RUN_LEVEL  = LOW;

// 연속 감지 후 모터를 시작할 시간
const unsigned long DETECT_TIME_MS = 1000;

unsigned long detectStartTime = 0;

bool detecting = false;
bool motorRunning = false;

void setMotor(bool run)
{
  // 상태가 실제로 변경될 때만 처리
  if (motorRunning == run)
  {
    return;
  }

  motorRunning = run;

  if (run)
  {
    digitalWrite(MOTOR_PIN, MOTOR_RUN_LEVEL);
    Serial.println("모터 구동 시작");
  }
  else
  {
    digitalWrite(MOTOR_PIN, MOTOR_STOP_LEVEL);
    Serial.println("모터 정지");
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  // 전원을 켰을 때 모터 정지
  digitalWrite(MOTOR_PIN, MOTOR_STOP_LEVEL);
  motorRunning = false;

  Serial.println("시스템 시작: IR 센서 감지 대기");
}

void loop()
{
  unsigned long currentTime = millis();

  bool detected =
    digitalRead(IR_SENSOR_PIN) == IR_DETECTED_LEVEL;

  if (detected)
  {
    // 최초 감지 시점 기록
    if (!detecting)
    {
      detecting = true;
      detectStartTime = currentTime;

      Serial.println("IR 감지 시작");
    }

    // 1초 동안 연속 감지되면 모터 구동
    if (!motorRunning &&
        currentTime - detectStartTime >= DETECT_TIME_MS)
    {
      setMotor(true);
    }
  }
  else
  {
    // 감지가 해제되면 즉시 모터 정지
    if (motorRunning)
    {
      Serial.println("IR 감지 해제");
      setMotor(false);
    }
    else if (detecting)
    {
      Serial.println("1초 전에 감지 해제");
    }

    // 감지 시간 초기화
    detecting = false;
    detectStartTime = 0;
  }
}