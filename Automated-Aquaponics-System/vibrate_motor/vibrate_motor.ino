const int IR_SENSOR_PIN = 2;
const int MOTOR_PIN = 9;

// 일반적인 IR 센서 모듈은 물체 감지 시 LOW
const int IR_DETECTED_LEVEL = LOW;

// 모터 PWM 입력
// LOW  = 정지
// HIGH = 최대 속도
const int MOTOR_STOP_LEVEL = HIGH;
const int MOTOR_RUN_LEVEL  = LOW;

const unsigned long DETECT_TIME_MS = 5000;  // 5초

unsigned long detectStartTime = 0;
bool detecting = false;
bool motorRunning = false;

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
}

void loop()
{
  bool detected =
      digitalRead(IR_SENSOR_PIN) == IR_DETECTED_LEVEL;

  if (detected)
  {
    // 처음 감지된 시점 기록
    if (!detecting)
    {
      detecting = true;
      detectStartTime = millis();

      Serial.println("IR 감지 시작");
    }

    // 1초 이상 연속 감지된 경우 모터 회전
    if (!motorRunning &&
        millis() - detectStartTime >= DETECT_TIME_MS)
    {
      setMotor(true);
      Serial.println("1초 연속 감지: 모터 회전");
    }
  }
  else
  {
    // 감지가 끊기면 시간 초기화 및 즉시 정지
    detecting = false;
    detectStartTime = 0;

    if (motorRunning)
    {
      setMotor(false);
      Serial.println("감지 해제: 모터 정지");
    }
  }
}