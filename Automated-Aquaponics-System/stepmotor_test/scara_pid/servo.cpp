#include "servo_config.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

static bool pwm_initialized = false;

static int degreeToPulseWidth(uint8_t degree)
{
  degree = constrain(degree, 0, 180);

  int pulse_wide = map(degree, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  int pulse_width = int(float(pulse_wide) / 1000000.0 * FREQUENCY * 4096);

  return pulse_width;
}

void grip_timer_on()
{
  if (!pwm_initialized) {
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);

    delay(20);   // PCA9685 PWM 안정화 시간
    pwm_initialized = true;
  }
}

void grip_timer_off()
{
  // PCA9685는 Servo.h처럼 detach할 필요가 없음
  // 필요하면 아래처럼 0번 채널 출력을 끌 수 있음
  // pwm.setPWM(SERVO_CHANNEL, 0, 0);
}

void smove(uint8_t degree)
{
  grip_timer_on();

  int pulse_width = degreeToPulseWidth(degree);
  pwm.setPWM(SERVO_CHANNEL, 0, pulse_width);
}

void setServo()
{
  smove(75); // 기본 상태
}

void grip(bool x)
{
  grip_timer_on();

  if (x == true) {
    smove(160);   // 열림
  } else {
    smove(80);   // 반대 상태
  }

  delay(300);     // 서보가 실제로 움직일 시간
}