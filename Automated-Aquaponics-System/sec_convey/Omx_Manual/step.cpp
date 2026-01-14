#include "step.h"

// OpenCR 예제 스타일: TIMER_CH1로 타이머 객체 생성
static HardwareTimer Timer(TIMER_CH1);

static uint8_t s_pul = 0, s_dir = 0, s_ena = 0;
static bool s_enaActiveLow = false;

static volatile bool s_pulseState = false;
static volatile uint32_t s_periodUs = 50; // 토글 주기(us)

void step_pin(){
  step_begin(4, 5, 6, false);

  step_enable(true);
  step_setDir(true);

  step_setStepsPerSec(300); // 300 steps/sec 시작
  step_start();
}

// ISR: PUL 핀 토글
static void step_isr()
{
  digitalWrite(s_pul, s_pulseState);
  s_pulseState = !s_pulseState;
}

void step_begin(uint8_t pulPin, uint8_t dirPin, uint8_t enaPin,
                bool enaActiveLow)
{
  s_pul = pulPin;
  s_dir = dirPin;
  s_ena = enaPin;
  s_enaActiveLow = enaActiveLow;

  pinMode(s_pul, OUTPUT);
  pinMode(s_dir, OUTPUT);
  pinMode(s_ena, OUTPUT);

  digitalWrite(s_pul, LOW);
  digitalWrite(s_dir, LOW);

  // 기본은 "비활성"로 시작 (원하면 step_enable(true) 호출)
  step_enable(false);

  Timer.pause();
  Timer.setPeriod(s_periodUs);     // us
  Timer.attachInterrupt(step_isr);
  Timer.refresh();
}

void step_enable(bool on)
{
  // ENA Active-Low이면 LOW가 enable, HIGH가 disable
  if (s_enaActiveLow) digitalWrite(s_ena, on ? LOW : HIGH);
  else                digitalWrite(s_ena, on ? HIGH : LOW);
}

void step_setDir(bool dir)
{
  digitalWrite(s_dir, dir ? HIGH : LOW);
}

void step_setTogglePeriodUs(uint32_t periodUs)
{
  if (periodUs < 2) periodUs = 2;        // 너무 짧으면 위험/불안정
  s_periodUs = periodUs;

  // 동작 중이면 즉시 반영
  Timer.setPeriod(s_periodUs);
}

void step_setStepsPerSec(float stepsPerSec)
{
  if (stepsPerSec < 1.0f) stepsPerSec = 1.0f;

  // ISR 토글이므로 "상승엣지(스텝)"는 2번 토글마다 1번 나옴
  // stepsPerSec = 1 / (2*period)  -> period(us) = 1e6 / (2*stepsPerSec)
  float period = 1000000.0f / (2.0f * stepsPerSec);
  step_setTogglePeriodUs((uint32_t)period);
}

void step_start()
{
  s_pulseState = false;
  digitalWrite(s_pul, LOW);
  Timer.refresh();
  Timer.resume();
}

void step_stop()
{
  Timer.pause();
  digitalWrite(s_pul, LOW);
}
