const int IR_SENSOR_PIN = 2;
const int MOTOR_PIN = 9;

// 일반적인 IR 센서 모듈은 물체 감지 시 LOW
const int IR_DETECTED_LEVEL = LOW;

// 현재 사용 중인 모터 입력 논리
const int MOTOR_STOP_LEVEL = HIGH;
const int MOTOR_RUN_LEVEL  = LOW;

const unsigned long DETECT_TIME_MS = 3000;   // 연속 감지 시간: 3초
const unsigned long MOTOR_RUN_MS   = 5000;  // 모터 구동 시간: 5초

unsigned long detectStartTime = 0;
unsigned long motorStartTime = 0;

bool detecting = false;
bool motorRunning = false;
bool cycleCompleted = false;

void setMotor(bool run)
{
  motorRunning = run;

  if (run)
  {
    digitalWrite(MOTOR_PIN, MOTOR_RUN_LEVEL);
  }
  else
  {
    digitalWrite(MOTOR_PIN, MOTOR_STOP_LEVEL);
  }
}

void setup()
{
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  // 전원을 켰을 때 모터 정지
  setMotor(false);

  Serial.begin(115200);
  Serial.println("시스템 시작");
}

void loop()
{
  unsigned long currentTime = millis();

  bool detected =
      digitalRead(IR_SENSOR_PIN) == IR_DETECTED_LEVEL;

  // 모터가 동작 중인 경우
  if (motorRunning)
  {
    // 10초가 지나면 모터 정지
    if (currentTime - motorStartTime >= MOTOR_RUN_MS)
    {
      setMotor(false);
      cycleCompleted = true;

      Serial.println("10초 구동 완료: 모터 정지");
    }

    return;
  }

  // 한 번의 구동이 완료된 경우
  if (cycleCompleted)
  {
    // 센서 감지가 해제되어야 다음 동작 가능
    if (!detected)
    {
      cycleCompleted = false;
      detecting = false;
      detectStartTime = 0;

      Serial.println("감지 해제: 다음 감지 대기");
    }

    return;
  }

  // 모터 정지 상태에서 센서 감지 확인
  if (detected)
  {
    if (!detecting)
    {
      detecting = true;
      detectStartTime = currentTime;

      Serial.println("IR 감지 시작");
    }

    // 4초간 연속 감지되면 모터 구동
    if (currentTime - detectStartTime >= DETECT_TIME_MS)
    {
      setMotor(true);
      motorStartTime = currentTime;

      Serial.println("4초 연속 감지: 모터 10초 구동 시작");
    }
  }
  else
  {
    // 4초가 되기 전에 감지가 끊기면 초기화
    if (detecting)
    {
      Serial.println("감지 중단: 감지 시간 초기화");
    }

    detecting = false;
    detectStartTime = 0;
  }
}