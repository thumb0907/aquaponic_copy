#include "encoder.h"
#include "set_motor.h"
#include "pid.h"
#include "servo_config.h"

float j2_pos_mm = 0.0f;
const float cpr    = 400.0f;
const float en_cnt = cpr * 2.0f;
const int j3_gear = 16;
const int j1_gear = 20;
const float j4_gear = 4.5;
volatile bool j1_run=false, j2_run=false, j3_run=false, j4_run=false;
static volatile bool j1_ps=false, j2_ps=false, j3_ps=false, j4_ps=false;

encod j1_enc = { j1_A, j1_B, 0 };
encod j3_enc = { j3_A, j3_B, 0 };
encod j4_enc = { j4_A, j4_B, 0 };

volatile bool j1pulseState = false;
volatile bool j3pulseState = false;
volatile bool j4pulseState = false;

volatile bool j2pulseState = false;
volatile long j2pulseInterval = 100;

volatile long j1pulseInterval = 100;
volatile long j3pulseInterval = 100;
volatile long j4pulseInterval = 100;

// z축 
const unsigned int PULSE_US = 5;   // 펄스폭은 넉넉히
const long PULSES_PER_REV = 800;  // (가정) 1.8°모터 + 128분주
float J2_LEAD_MM_PER_REV = (8.0f/1.0f);
unsigned long J2_DEFAULT_PPS =3500;   // j2 기본 속도
volatile bool j2_endstop_hit = false;

constexpr bool EN_ACTIVE_LOW = false; // 모터enable

volatile long j2_target_steps = 0;
volatile long j2_step_count   = 0;
volatile bool j2_move_done    = false;

//리니어 레일
volatile bool rail_run = false;
volatile bool railPulseState = false;
volatile long railPulseInterval = 100;

static bool waitStableLow(uint8_t pin, unsigned long stable_ms);
static bool waitStableHigh(uint8_t pin, unsigned long stable_ms, unsigned long timeout_ms);

float encoder_getAngleDeg(const encod* e) {
  return -(e->pos) * 360.0f / en_cnt;
}

void set_tim()
{
  Timer1.initialize(50);
  Timer1.attachInterrupt(j1stepPulse);

  Timer4.initialize(50);
  Timer4.attachInterrupt(j3stepPulse);

  Timer3.initialize(50);
  Timer3.attachInterrupt(railStepPulse);   // 기본값
  Timer3.stop();                           // 평소엔 정지

  Timer5.initialize(50);
  Timer5.attachInterrupt(j4stepPulse);
}

void set_int()
{
  attachInterrupt(digitalPinToInterrupt(j1_enc.pinA), j1EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j3_enc.pinA), j3EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j4_enc.pinA), j4EncoderA, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(stop_z), j2EndstopISR, FALLING);
}


//관절 1에 대한 함수들 //////////////////////////////////////////////////////////////////////
void j1stepPulse() {
  if(!j1_run){ 
    digitalWrite(j1_pul, LOW); 
    return; 
  }
  digitalWrite(j1_pul, j1pulseState);
  j1pulseState = !j1pulseState;
}


void j1EncoderA() {//엔코더 읽기
  bool a = digitalRead(j1_enc.pinA);
  bool b = digitalRead(j1_enc.pinB);
  j1_enc.pos += (a == b) ? 1 : -1;
}

bool move_j1(float targetAngle, unsigned long max_pps, float tolDeg)
{
  float Angle = j1_gear * targetAngle;

  j1_run = true;

  noInterrupts();
  long Count = j1_enc.pos;
  interrupts();

  encod snap = j1_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = Angle - nowAngle;
  float pidOut = pid_update(&j1_pid, error);

  if (max_pps < 1) max_pps = 1;
  if(max_pps > J1_DEFAULT_MAX_PPS) max_pps = J1_DEFAULT_MAX_PPS;

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > max_pps) speed = max_pps;

  long interval = 1000000L / (2.0f * speed);
  
  noInterrupts();
  j1pulseInterval = interval;
  Timer1.setPeriod(j1pulseInterval);
  interrupts();

  digitalWrite(j1_dir, (pidOut > 0) ? LOW : HIGH);
  /*
  Serial.print("Angle="); Serial.print(nowAngle);
  Serial.print(" Error="); Serial.print(error);
  Serial.print(" speed="); Serial.print(speed);
  Serial.print(" interval(us)="); Serial.println(interval);
*/
  if (fabs(error) <= j1_gear * tolDeg) {
    j1_run = false;              
    digitalWrite(j1_pul, LOW);   
    return true;                 
  }
  return false;  
}

static float j1_now_deg() // 현재 각도값 저장하는 함수
{
  noInterrupts();
  long c = j1_enc.pos;
  interrupts();

  encod snap = j1_enc;
  snap.pos = c;
  return encoder_getAngleDeg(&snap);
}

static float j1_error_deg(float targetAngle) //현재 오차값 저장하는 함수
{
  return (j1_gear * targetAngle) - j1_now_deg();
}

bool move_j1_wait(float targetAngle,
                  unsigned long max_pps,
                  float tolDeg,
                  unsigned long stable_ms,
                  unsigned long timeout_ms)
{
  unsigned long t0 = millis();
  unsigned long inTolSince = 0;

  motors_enable_all(true);

  while (true) {
    move_j1(targetAngle, max_pps, tolDeg);

    float e = j1_error_deg(targetAngle);
    if (fabs(e) <j1_gear* tolDeg) {
      if (inTolSince == 0) inTolSince = millis();
      if (millis() - inTolSince >= stable_ms) break;
    } else {
      inTolSince = 0;
    }

    if (millis() - t0 > timeout_ms) break;
  }

  // 정지(필요하면)
  j1_run = false;
  digitalWrite(j1_pul, LOW);
  motors_enable_all(false);
  return true;
}

static void j1_set_pps(unsigned long pps) {
  if (pps < 1) pps = 1;
  unsigned long isr_us = 1000000UL / (2 * pps);  // 토글이라 2배
  //if (isr_us < 50) isr_us = 50;                  // 너무 빠른 값 방지(필요시 조절)
  noInterrupts();
  Timer1.setPeriod(isr_us);
  interrupts();
  j1_run = true;
}
void enc_reset_j1()
{
  noInterrupts();
  j1_enc.pos = 0;
  interrupts();
}

void j1_home_stop_on_switch(bool dir_to_switch, unsigned long pps)
{
  motors_enable_all(true);
  pinMode(stop_j1, INPUT_PULLUP);

  auto setDirToward = [&](bool toward){
    digitalWrite(j1_dir, toward ? HIGH : LOW);
  };

  setDirToward(dir_to_switch);
  j1_set_pps(pps);
  j1_run = true;

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 5000;
  const unsigned long STABLE_HIGH_MS = 20;

  while (true) {
    if (millis() - t0 > TIMEOUT_MS) {
      j1_run = false;
      digitalWrite(j1_pul, LOW);
      motors_enable_all(false);
      return;
    }

    if (digitalRead(stop_j1) == HIGH) {
      waitStableHigh(stop_j1, STABLE_HIGH_MS, 1000);
      break;
    }

    delay(1);
  }

  j1_run = false;
  digitalWrite(j1_pul, LOW);

  enc_reset_j1();
  motors_enable_all(false);
}

///////////////////////////////////////////////////////////////////////////////////////////////

//관절 2에 대한 함수들 /////////////////////////////////////////////////////////////////////////
static inline void j2_enable(bool on)
{
  pinMode(j2_en, OUTPUT);
  digitalWrite(j2_en, on ? LOW : HIGH);
  // active level이 반대면 HIGH/LOW 뒤집으십시오.
}

void j2stepPulse()
{
  if (!j2_run) {
    digitalWrite(STEP_PIN, LOW);
    return;
  }

  digitalWrite(STEP_PIN, j2pulseState);

  // 상승엣지에서만 1 step 카운트
  if (j2pulseState == HIGH) {
    j2_step_count++;

    if (j2_target_steps > 0 && j2_step_count >= j2_target_steps) {
      j2_run = false;
      j2_move_done = true;
      j2pulseState = LOW;
      digitalWrite(STEP_PIN, LOW);
      return;
    }
  }
  j2pulseState = !j2pulseState;
}

void j2_set_pps(unsigned long pps)
{
  if (pps < 1) pps = 1;

  unsigned long isr_us = 1000000UL / (2UL * pps);
  if (isr_us < 50) isr_us = 50;

  noInterrupts();
  j2pulseInterval = isr_us;
  Timer3.setPeriod(j2pulseInterval);
  interrupts();
}

void move_j2_continuous(bool dir, unsigned long pps)
{
  if (rail_run) {
    //Serial.println("[J2] blocked: rail is running");
    return;
  }

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(j2_en, OUTPUT);

  j2_enable(true);

  //Serial.print("[J2 DIR] input dir = ");
  //Serial.println(dir ? "true" : "false");

  digitalWrite(DIR_PIN, dir ? LOW : HIGH);   // 필요시 HIGH/LOW 뒤집기
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(50);                     // 추가

  noInterrupts();
  j2pulseState    = LOW;
  j2_target_steps = 0;
  j2_step_count   = 0;
  j2_move_done    = false;
  j2_run          = false;
  interrupts();

  Timer3.detachInterrupt();
  Timer3.attachInterrupt(j2stepPulse);

  j2_set_pps(pps);

  noInterrupts();
  j2_run = true;
  interrupts();

  Timer3.start();
}

bool j2_home_stop_on_switch(bool toward_home, unsigned long pps)
{
  if (rail_run) {
    //Serial.println("[J2 HOME] blocked: rail is running");
    return false;
  }

  // 이미 스위치 눌린 상태면 먼저 조금 빠져나오기
  if (is_j2_endstop_pressed()) {
    move_j2_continuous(!toward_home, pps / 2);
    while (is_j2_endstop_pressed()) {
      // 스위치에서 벗어날 때까지 대기
    }
    stop_j2_motion();
    delay(50);
  }

  // 홈 방향으로 연속 이동 시작
  move_j2_continuous(toward_home, pps);

  while (true) {
    if (is_j2_endstop_pressed()) {
      stop_j2_motion();
      delay(50);
      return true;
    }
  }
}

bool j2_home_precise(bool toward_home, unsigned long fast_pps, unsigned long slow_pps)
{
  if (!j2_home_stop_on_switch(toward_home, fast_pps)) {
    return false;
  }

  // 1cm 백오프를 degree로 환산
  float backoff_mm  = toward_home ? -10.0f : 10.0f;
  float backoff_deg = backoff_mm * 360.0f / J2_LEAD_MM_PER_REV;
  move_j2(backoff_deg, slow_pps);
  delay(50);

  // 천천히 재접근
  if (!j2_home_stop_on_switch(toward_home, slow_pps)) {
    return false;
  }

  return true;
}
bool is_j2_endstop_pressed()
{
  const int check_count = 10;

  for (int i = 0; i < check_count; i++) {
    if (digitalRead(stop_z) != LOW) {
      return false;
    }
    delayMicroseconds(300);
  }
  return true;
}

bool move_j2(float deg, unsigned long pps)
{
  if (rail_run) {
    //Serial.println("[J2] blocked: rail is running");
    return false;
  }

  long steps = (long)(fabs(deg) * (float)PULSES_PER_REV / 360.0f);
  bool dir = (deg >= 0);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(j2_en, OUTPUT);

  Timer3.stop();

  j2_enable(true);
  digitalWrite(DIR_PIN, dir ? LOW : HIGH);
  delayMicroseconds(20);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(20);

  noInterrupts();
  j2pulseState    = LOW;
  j2_step_count   = 0;
  j2_target_steps = steps;
  j2_move_done    = false;
  j2_run          = false;
  interrupts();

  Timer3.detachInterrupt();
  Timer3.attachInterrupt(j2stepPulse);

  j2_set_pps(pps);

  noInterrupts();
  j2_run = true;
  interrupts();

  Timer3.start();

  while (true) {
    // dir == true 가 홈 방향이라고 가정
    if (dir && is_j2_endstop_pressed()) {
      stop_j2_motion();
      return false;
    }

    if (j2_move_done) {
      stop_j2_motion();
      return true;
    }
  }
}
void stop_j2_motion(bool disable_after)
{
  noInterrupts();
  j2_run          = false;
  j2pulseState    = LOW;
  j2_target_steps = 0;
  j2_step_count   = 0;
  j2_move_done    = false;
  interrupts();

  digitalWrite(STEP_PIN, LOW);
  Timer3.stop();

  if (disable_after) {
    j2_enable(false);
  }
}

void j2EndstopISR(){
  j2_endstop_hit = true;   // ISR은 플래그만!
}
/////////////////////////////////////////////////////////////////////////////////////////////

//관절 3에 대한 함수들////////////////////////////////////////////////////////////////////////
void j3stepPulse() {//펄스 생성 코드
   if(!j3_run){ 
    digitalWrite(j3_pul, LOW); 
    return; 
   }            
  digitalWrite(j3_pul, j3pulseState);
  j3pulseState = !j3pulseState;
}
void j3EncoderA() {
  bool a = digitalRead(j3_enc.pinA);
  bool b = digitalRead(j3_enc.pinB);
  j3_enc.pos += (a == b) ? 1 : -1;
}

bool move_j3(float targetAngle, unsigned long max_pps, float tolDeg)
{
  float Angle = j3_gear * targetAngle;

  j3_run = true;

  noInterrupts();
  long Count = j3_enc.pos;
  interrupts();

  encod snap = j3_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = Angle - nowAngle;
  float pidOut = pid_update(&j3_pid, error);
  
  if (fabs(error) <= j3_gear * tolDeg) {
    noInterrupts();
    j3_run = false;
    j3pulseState = LOW;
    digitalWrite(j3_pul, LOW);
    Timer4.stop();
    motors_enable_all(false);
    interrupts();
    return true;
  }

  if (max_pps < 1) max_pps = 1;
  if(max_pps > J3_DEFAULT_MAX_PPS) max_pps = J3_DEFAULT_MAX_PPS;

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > max_pps) speed = max_pps;

  long interval = 1000000L / (2.0f * speed);

  noInterrupts();
  j3pulseInterval = interval;
  Timer4.setPeriod(j3pulseInterval);
  interrupts();

  //Serial.print("Angle="); Serial.print(nowAngle);
  //Serial.print(" Error="); Serial.println(error);

  digitalWrite(j3_dir, (pidOut > 0) ? LOW : HIGH);
  return false;  
}
static float j3_now_deg() // 현재 각도값 저장하는 함수
{
  noInterrupts();
  long c = j3_enc.pos;
  interrupts();

  encod snap = j3_enc;
  snap.pos = c;
  return encoder_getAngleDeg(&snap);
}

static float j3_error_deg(float targetAngle) //현재 오차값 저장하는 함수
{
  return (j3_gear * targetAngle) - j3_now_deg();
} 
bool move_j3_wait(
  float targetAngle,
  unsigned long max_pps,
  float tolDeg,
  unsigned long stable_ms,
  unsigned long timeout_ms
) {
  const unsigned long startedAt = millis();
  unsigned long inToleranceSince = 0;

  bool completed = false;

  motors_enable_all(true);

  while (true) {
    const bool reached = move_j3(
      targetAngle,
      max_pps,
      tolDeg
    );

    if (reached) {
      if (inToleranceSince == 0) {
        inToleranceSince = millis();
      }

      if (
        millis() - inToleranceSince
        >= stable_ms
      ) {
        completed = true;
        break;
      }
    }
    else {
      inToleranceSince = 0;
    }

    if (
      millis() - startedAt
      >= timeout_ms
    ) {
      completed = false;
      break;
    }
  }

  noInterrupts();
  j3_run = false;
  j3pulseState = LOW;
  digitalWrite(j3_pul, LOW);
  interrupts();

  motors_enable_all(false);

  return completed;
}

static void j3_set_pps(unsigned long pps) {
  if (pps < 1) pps = 1;
  unsigned long isr_us = 1000000UL / (2 * pps);  // 토글이라 2배
  if (isr_us < 50) isr_us = 50;                  // 너무 빠른 값 방지(필요시 조절)
  noInterrupts();
  Timer4.setPeriod(isr_us);
  interrupts();
  j3_run = true;
}

// stop 핀에서 LOW가 stable_ms 동안 "연속" 유지될 때만 true
static bool waitStableLow(uint8_t pin, unsigned long stable_ms)
{
  unsigned long t_start = millis();

  while (millis() - t_start < stable_ms) {
    if (digitalRead(pin) != LOW) {
      // 중간에 HIGH가 한번이라도 나오면 다시 처음부터
      t_start = millis();
    }
    //delay(1); // 1ms 샘플링(너무 빡세게 돌리면 오히려 노이즈에 취약)
  }
  return true;
}

static bool waitStableHigh(uint8_t pin, unsigned long stable_ms, unsigned long timeout_ms = 1000)
{
  unsigned long t0 = millis();
  unsigned long t_start = millis();

  while (true) {
    if (digitalRead(pin) == HIGH) {
      if (millis() - t_start >= stable_ms) return true;
    } else {
      t_start = millis();
    }

    if (millis() - t0 > timeout_ms) return false;
    //delay(1);
  }
}
void enc_reset_j3()
{
  noInterrupts();
  j3_enc.pos = 0;
  interrupts();
}
bool is_j3_endstop_pressed()
{
  const int check_count = 6;

  for (int i = 0; i < check_count; i++) {
    if (digitalRead(stop_j3) != LOW) {
      return false;
    }
    delayMicroseconds(200);
  }
  return true;
}
void j3_home_stop_on_switch(bool dir_to_switch, unsigned long pps)
{
  motors_enable_all(true);
  pinMode(stop_j3, INPUT_PULLUP);

  auto setDirToward = [&](bool toward){
    digitalWrite(j3_dir, toward ? HIGH : LOW);
  };

  setDirToward(dir_to_switch);

  noInterrupts();
  j3pulseState = LOW;
  j3_run = false;
  interrupts();

  j3_set_pps(pps);
  j3_run = true;

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 5000;
  const unsigned long STABLE_LOW_MS = 20;

  while (true) {
    if (millis() - t0 > TIMEOUT_MS) {
      noInterrupts();
      j3_run = false;
      j3pulseState = LOW;
      digitalWrite(j3_pul, LOW);
      interrupts();
      motors_enable_all(false);
      return;
    }

    if (digitalRead(stop_j3) == LOW) {
      waitStableLow(stop_j3, STABLE_LOW_MS);
      break;
    }
  }

  noInterrupts();
  j3_run = false;
  j3pulseState = LOW;
  digitalWrite(j3_pul, LOW);
  interrupts();

  enc_reset_j3();
  motors_enable_all(false);
}
////////////////////////////////////////////////////////////////////////////////////////////

//관절 4에 대한 함수들//////////////////////////////////////////////////////////////////////
volatile long j4_step_edges = 0;

void j4stepPulse()
{
  if (!j4_run) {
    digitalWrite(j4_pul, LOW);
    return;
  }

  digitalWrite(j4_pul, j4pulseState);
  j4pulseState = !j4pulseState;
}

static void j4_set_pps(unsigned long pps)
{
  if (pps < 1) pps = 1;

  unsigned long isr_us = 1000000UL / (2UL * pps);

  // Timer5 initialize가 50us 기준이라 너무 작은 값은 막음
  if (isr_us < 50) isr_us = 50;

  noInterrupts();
  j4pulseInterval = isr_us;
  Timer5.setPeriod(j4pulseInterval);
  interrupts();
}
/*
void j4stepPulse() {

  if(!j4_run){
    digitalWrite(j4_pul, LOW);
    return;
  }

  digitalWrite(j4_pul, j4pulseState);

  // 상승엣지(LOW->HIGH)에서만 1 step으로 카운트
  if (j4pulseState == HIGH) {
    j4_step_edges++;

    // ===== open-loop용 step count =====
    j4_ol_step_count++;

    if (j4_ol_target_steps > 0 && j4_ol_step_count >= j4_ol_target_steps) {
      j4_run = false;
      j4_ol_move_done = true;
      j4pulseState = LOW;
      digitalWrite(j4_pul, LOW);
      return;
    }
  }

  j4pulseState = !j4pulseState;
}
*/
void j4EncoderA() {
  bool a = digitalRead(j4_enc.pinA);
  bool b = digitalRead(j4_enc.pinB);
  j4_enc.pos += (a == b) ? 1 : -1;
}

bool move_j4(float targetAngle, float tolDeg = 0.3f)
{
  float Angle = j4_gear * targetAngle;

  j4_run = true;

  noInterrupts();
  long Count = j4_enc.pos;
  interrupts();

  encod snap = j4_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error = Angle - nowAngle;
  float pidOut = pid_update(&j4_pid, error);

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 2000) speed = 2000;

  long interval = 1000000L / (2.0f * speed);

  noInterrupts();
  j4pulseInterval = interval;
  Timer5.setPeriod(j4pulseInterval);
  interrupts();

  digitalWrite(j4_dir, (pidOut > 0) ? LOW : HIGH);

  //Serial.print("Angle="); Serial.print(nowAngle);
  //Serial.print(" Error="); Serial.println(error);
  //Serial.print(" speed="); Serial.print(speed);
  //Serial.print(" interval(us)="); Serial.println(interval);

  if (fabs(error) <= j4_gear * tolDeg) {
    j4_run = false;              
    digitalWrite(j4_pul, LOW);   
    return true;                 
  }
  return false;  
}

static float j4_now_deg() // 현재 각도값 저장하는 함수
{
  noInterrupts();
  long c = j4_enc.pos;
  interrupts();

  encod snap = j4_enc;
  snap.pos = c;
  return encoder_getAngleDeg(&snap);
}

static float j4_error_deg(float targetAngle) //현재 오차값 저장하는 함수
{
  return (j4_gear * targetAngle) - j4_now_deg();
} 

void move_j4_wait(float targetAngle,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 8000)
{
  unsigned long t0 = millis();
  unsigned long inTolSince = 0;

  motors_enable_all(true);

  while (true) {
    move_j4(targetAngle, tolDeg);

    float e = j4_error_deg(targetAngle);
    if (fabs(e) <= j4_gear * tolDeg) {
      if (inTolSince == 0) inTolSince = millis();
      if (millis() - inTolSince >= stable_ms) break;
    } else {
      inTolSince = 0;
    }

    if (millis() - t0 > timeout_ms) break;

    //delay(5);
  }

  j4_run = false;
  digitalWrite(j4_pul, LOW);

  motors_enable_all(false);
}

/*
static void j4_set_pps(unsigned long pps) { 
  if (pps < 1) pps = 1; unsigned long isr_us = 1000000UL / (2 * pps); // 토글이라 2배 if (isr_us < 50) isr_us = 50; // 너무 빠른 값 방지(필요시 조절)
  noInterrupts(); 
  Timer5.setPeriod(isr_us); 
  interrupts();
  j4_run = true; 
}*/

bool j4_home_stop_on_switch_safe(bool dir_to_switch, unsigned long pps)
{
  motors_enable_all(true);

  auto setDirToward = [&](bool toward){
    digitalWrite(j4_dir, toward ? HIGH : LOW);
  };

  // 1) 스위치 쪽으로 접근(눌릴 때까지), timeout 필수
  setDirToward(dir_to_switch);
  
  noInterrupts();
  j4pulseState = LOW;
  j4_run = false;
  interrupts();
  
  j4_set_pps(pps);
  j4_run = true;
  
  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 5000;

  while (true) {
    if (millis() - t0 > TIMEOUT_MS) {
      noInterrupts();
      j4_run = false;
      j4pulseState = LOW;
      digitalWrite(j4_pul, LOW);
      interrupts();
      motors_enable_all(false);
      return false;
    }

    if (digitalRead(stop_j4) == LOW) {
      noInterrupts();
      j4_run = false;
      j4pulseState = LOW;
      digitalWrite(j4_pul, LOW);
      interrupts();
      break;
    }
  }

  //delay(20);   // 필요하면 정지 후 안정화 대기
  enc_reset_j4();
  motors_enable_all(false);
  return true;
}


void enc_reset_j4()
{
  noInterrupts();
  j4_enc.pos = 0;
  interrupts();
}
////////////////////////////////////////////////////////////////////////////////

// 스텝모터 enable
static inline void j1_enable(bool on)
{
  pinMode(j1_en, OUTPUT);
  digitalWrite(j1_en, on ? (EN_ACTIVE_LOW ? LOW : HIGH)
                        : (EN_ACTIVE_LOW ? HIGH : LOW));
}
static inline void j3_enable(bool on)
{
  pinMode(j3_en, OUTPUT);
  digitalWrite(j3_en, on ? (EN_ACTIVE_LOW ? LOW : HIGH)
                        : (EN_ACTIVE_LOW ? HIGH : LOW));
}
static inline void j4_enable(bool on)
{
  pinMode(j4_en, OUTPUT);
  digitalWrite(j4_en, on ? (EN_ACTIVE_LOW ? HIGH : LOW)
                        : (EN_ACTIVE_LOW ? LOW : HIGH));
}
static inline void motors_enable_all(bool on) // 전체 en
{
  j1_enable(on);
  j3_enable(on);
  //j4_enable(on);
  //pinMode(j2_en, OUTPUT);
  digitalWrite(j4_en, on ? (EN_ACTIVE_LOW ? LOW : HIGH)
                        : (EN_ACTIVE_LOW ? HIGH : LOW));
}


////////////////////////////////////////////////////////////////////////////

//========조인트 값 받기=================================================

float j1_getJointDeg()
{
  noInterrupts();
  long c = j1_enc.pos;
  interrupts();

  encod snap = j1_enc;
  snap.pos = c;

  float motor_deg = encoder_getAngleDeg(&snap); // 모터축 각도(기존 로직)
  return motor_deg / (float)j1_gear;            // 조인트각 = 모터각 / 기어비
}

float j3_getJointDeg()
{
  noInterrupts();
  long c = j3_enc.pos;
  interrupts();

  encod snap = j3_enc;
  snap.pos = c;

  float motor_deg = encoder_getAngleDeg(&snap);
  return motor_deg / (float)j3_gear;
}

float j4_getJointDeg()
{
  noInterrupts();
  long c = j4_enc.pos;
  interrupts();

  encod snap = j4_enc;
  snap.pos = c;

  float motor_deg = encoder_getAngleDeg(&snap); // 모터축 각도(기존 로직)
  return motor_deg / (float)j4_gear;            // 조인트각 = 모터각 / 기어비
}
//===========================================================================

void enc_reset_all()
{
  noInterrupts();
  j1_enc.pos = 0;
  j3_enc.pos = 0;
  j4_enc.pos = 0;
  interrupts();
}

// ================== rail pulse ISR ==================
void railStepPulse()
{
  if (!rail_run) {
    digitalWrite(rail_pul, LOW);
    return;
  }

  digitalWrite(rail_pul, railPulseState);
  railPulseState = !railPulseState;
}

// ================== rail 속도 설정 ==================
void rail_set_pps(unsigned long pps)
{
  if (pps < 1) pps = 1;

  unsigned long isr_us = 1000000UL / (2UL * pps);  // 토글 기반이라 2배
  if (isr_us < 50) isr_us = 50;                    // 너무 빠른 값 방지

  noInterrupts();
  railPulseInterval = isr_us;
  Timer3.setPeriod(railPulseInterval);
  interrupts();
}

// ================== rail 시작 ==================
void moveRail(unsigned long pps, bool dir)
{
  if (j2_run) {
    //Serial.println("[RAIL] blocked: j2 is running");
    return;
  }

  pinMode(rail_pul, OUTPUT);
  pinMode(rail_dir, OUTPUT);
  pinMode(rail_en, OUTPUT);

  digitalWrite(rail_dir, dir ? HIGH : LOW);
  digitalWrite(rail_en, HIGH);
  digitalWrite(rail_pul, LOW);

  noInterrupts();
  railPulseState = LOW;
  rail_run = false;
  interrupts();

  Timer3.detachInterrupt();
  Timer3.attachInterrupt(railStepPulse);

  rail_set_pps(pps);

  noInterrupts();
  rail_run = true;
  interrupts();

  Timer3.start();
}

void stopRail(bool disable_after)
{
  noInterrupts();
  rail_run = false;
  railPulseState = LOW;
  interrupts();

  digitalWrite(rail_pul, LOW);
  Timer3.stop();

  if (disable_after) {
    digitalWrite(rail_en, LOW);
  }
}

bool is_rail_endstop_pressed(uint8_t pin)
{
  const int check_count = 20;

  for (int i = 0; i < check_count; i++) {
    if (digitalRead(pin) != HIGH) {
      return false;
    }
    delayMicroseconds(300);
  }
  return true;
}

bool moveRail_untilStop(bool dir, unsigned long pps,
                        uint8_t stop_pin = stop_rail,
                        unsigned long timeout_ms = 80000)
{
  pinMode(stop_pin, INPUT_PULLUP);

  // 이미 눌린 상태면 먼저 백오프
  if (is_rail_endstop_pressed(stop_pin)) {
    stopRail(true);
    //Serial.println("[RAIL] endstop already pressed -> backoff");

    if (stop_pin == stop_rail) {
      moveRail(pps, false);   // stop_rail에서는 false 방향으로 500ms
      delay(300);
      stopRail(true);
    }
    else if (stop_pin == stop2_rail) {
      moveRail(pps, true);    // stop2_rail에서는 true 방향으로 500ms
      delay(300);
      stopRail(true);
    }
    else if (stop_pin == stop3_rail) {
      moveRail(pps, true);
      delay(300);
      stopRail(true);
    }
    else {
      stopRail(true);
      return false;
    }

    delay(100);  // 안정화
    //Serial.println("[RAIL] backoff done -> retry move");
  }

  // 원래 목표 방향으로 다시 이동
  moveRail(pps, dir);

  unsigned long t0 = millis();

  while (true) {
    if (is_rail_endstop_pressed(stop_pin)) {
      stopRail(true);
      delay(50);
      //Serial.println("[RAIL] endstop hit");
      return true;
    }

    if (millis() - t0 > timeout_ms) {
      stopRail(true);
      //Serial.println("[RAIL] timeout");
      return false;
    }
  }
}

/*
// ========================= J4 open-loop (엔코더 미사용) =========================

// J4 enable 제어 (현재 네 보드 기준 유지)
static inline void j4_enable_openloop(bool on)
{
  pinMode(j4_en, OUTPUT);
  digitalWrite(j4_en, on ? HIGH : LOW);
}

void j4_set_home_zero_deg(float deg)
{
  j4_openloop_deg = deg;
}

float j4_get_openloop_deg()
{
  return j4_openloop_deg;
}

void stop_j4_openloop(bool disable_after)
{
  noInterrupts();
  j4_run             = false;
  j4pulseState       = LOW;
  j4_ol_target_steps = 0;
  j4_ol_step_count   = 0;
  j4_ol_move_done    = false;
  interrupts();

  digitalWrite(j4_pul, LOW);
  Timer5.stop();

  if (disable_after) {
    j4_enable_openloop(false);
  }
}

static void j4_set_pps_openloop(unsigned long pps)
{
  if (pps < 1) pps = 1;

  unsigned long isr_us = 1000000UL / (2UL * pps);
  if (isr_us < 50) isr_us = 50;

  noInterrupts();
  j4pulseInterval = isr_us;
  Timer5.setPeriod(j4pulseInterval);
  interrupts();
}

// 홈 스위치 기준 원점 복귀 후 open-loop 각도 0도로 설정
bool j4_home_openloop(bool dir_to_switch, unsigned long pps = 1200)
{
  j4_enable_openloop(true);
  pinMode(stop_j4, INPUT_PULLUP);

  digitalWrite(j4_dir, dir_to_switch ? HIGH : LOW);
  digitalWrite(j4_pul, LOW);

  noInterrupts();
  j4pulseState       = LOW;
  j4_ol_target_steps = 0;
  j4_ol_step_count   = 0;
  j4_ol_move_done    = false;
  j4_run             = false;
  interrupts();

  j4_set_pps_openloop(pps);

  noInterrupts();
  j4_run = true;
  interrupts();

  Timer5.start();

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS    = 5000;
  const unsigned long STABLE_LOW_MS = 20;

  while (true) {
    if (millis() - t0 > TIMEOUT_MS) {
      stop_j4_openloop(false);
      return false;
    }

    if (digitalRead(stop_j4) == LOW) {
      waitStableLow(stop_j4, STABLE_LOW_MS);
      stop_j4_openloop(false);

      j4_openloop_deg = 0.0f;
      return true;
    }
  }
}

// 목표 "조인트각" 기준 절대 이동
bool move_j4_openloop(float target_deg, unsigned long pps)
{
  float delta_deg = target_deg - j4_openloop_deg;
  return move_j4_openloop_rel(delta_deg, pps);
}

// 목표 "조인트각" 기준 상대 이동
bool move_j4_openloop_rel(float delta_deg, unsigned long pps)
{
  // 조인트각 -> 모터축 스텝수로 변환
  long steps = (long)(fabs(delta_deg) * (float)j4_gear * (float)PULSES_PER_REV / 360.0f);
  bool dir = (delta_deg >= 0);

  if (steps <= 0) {
    return true;
  }

  j4_enable_openloop(true);

  pinMode(j4_pul, OUTPUT);
  pinMode(j4_dir, OUTPUT);
  pinMode(j4_en, OUTPUT);
  pinMode(stop_j4, INPUT_PULLUP);

  // 방향은 기존 J4 방향계에 맞춰 필요시 반대로 바꾸면 됨
  digitalWrite(j4_dir, dir ? LOW : HIGH);
  digitalWrite(j4_pul, LOW);
  delayMicroseconds(20);

  noInterrupts();
  j4pulseState       = LOW;
  j4_ol_step_count   = 0;
  j4_ol_target_steps = steps;
  j4_ol_move_done    = false;
  j4_run             = false;
  interrupts();

  j4_set_pps_openloop(pps);

  noInterrupts();
  j4_run = true;
  interrupts();

  Timer5.start();

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 8000;

  while (true) {
    // 홈 방향으로 이동 중 엔드스탑 재충돌 방지
    if (dir && digitalRead(stop_j4) == LOW) {
      stop_j4_openloop(false);
      return false;
    }

    // ISR 대신 여기서 step edge 수를 직접 기준 삼으려면 안 됨.
    // j4stepPulse()를 아래처럼 살짝 보강해야 함.
    if (j4_ol_move_done) {
      stop_j4_openloop(false);

      if (dir) j4_openloop_deg += fabs(delta_deg);
      else     j4_openloop_deg -= fabs(delta_deg);

      return true;
    }

    if (millis() - t0 > TIMEOUT_MS) {
      stop_j4_openloop(false);
      return false;
    }
  }
}*/
void j4_move(bool dir, unsigned long pps)
{
  //delay(100);
  pinMode(j4_dir, OUTPUT);
  delay(10);
  pinMode(j4_pul, OUTPUT);
  pinMode(j4_en, OUTPUT);
  pinMode(stop_j4, INPUT_PULLUP);

  delay(100);
  
  if (pps < 100)  pps = 100;
  if (pps > 3000) pps = 3000;

  unsigned long isr_us = 1000000UL / (2UL * pps);
  if (isr_us < 50) isr_us = 50;

  noInterrupts();
  j4pulseState = LOW;
  j4_run = false;
  interrupts();

  digitalWrite(j4_pul, LOW);
  digitalWrite(j4_dir, dir ? HIGH : LOW);
  digitalWrite(j4_en, LOW);

  Timer5.stop();
  Timer5.detachInterrupt();
  Timer5.initialize(isr_us);
  Timer5.attachInterrupt(j4stepPulse);

  noInterrupts();
  j4_run = true;
  interrupts();

  Timer5.start();
}

void j4_stop()
{
  noInterrupts();
  j4_run = false;
  j4pulseState = LOW;
  interrupts();

  digitalWrite(j4_pul, LOW);

  Timer5.stop();
  Timer5.detachInterrupt();   // J4가 Timer5 인터럽트 소유 해제

  digitalWrite(j4_en, LOW);   // 토크 유지

  grip_timer_on();            // J4 종료 후 Servo가 다시 Timer5 소유
}

void j4_home()
{
  pinMode(stop_j4, INPUT_PULLUP);

  const bool HOME_DIR = true;          // 방향 반대면 false로 변경
  const unsigned long HOME_PPS = 1200;
  const unsigned long TIMEOUT_MS = 8000;
  const unsigned long STABLE_LOW_MS = 20;

  // 이미 스위치가 눌려 있으면 반대 방향으로 살짝 빠짐
  if (digitalRead(stop_j4) == LOW) {
    j4_move(!HOME_DIR, HOME_PPS / 2);

    unsigned long t_back = millis();
    while (digitalRead(stop_j4) == LOW) {
      if (millis() - t_back > 1000) {
        j4_stop();
        return;
      }
    }

    j4_stop();
    delay(50);
  }

  // 홈 방향으로 이동
  j4_move(HOME_DIR, HOME_PPS);

  unsigned long t0 = millis();
  unsigned long low_start = 0;

  while (true) {
    if (digitalRead(stop_j4) == LOW) {
      if (low_start == 0) {
        low_start = millis();
      }

      if (millis() - low_start >= STABLE_LOW_MS) {
        j4_stop();
        return;
      }
    } else {
      low_start = 0;
    }

    if (millis() - t0 > TIMEOUT_MS) {
      j4_stop();
      return;
    }
  }
}
// ============================================================================
// J1 / J3 Open-loop 함수
// - 엔코더 미사용
// - 기존 move_j1(), move_j3(), j1stepPulse(), j3stepPulse() 수정 없음
// - Timer1, Timer4를 임시로 빌려 쓰고, stop 시 기존 ISR로 복구
// ============================================================================

// -------------------- J1 open-loop state --------------------
static volatile bool j1_ol_run = false;
static volatile bool j1_ol_pulse_state = LOW;
static volatile long j1_ol_target_steps = 0;
static volatile long j1_ol_step_count = 0;
static volatile bool j1_ol_done = false;

static void j1_ol_set_pps(unsigned long pps)
{
  if (pps < 1) pps = 1;

  unsigned long isr_us = 1000000UL / (2UL * pps);

  // 너무 빠른 인터럽트 방지
  if (isr_us < 50) isr_us = 50;

  noInterrupts();
  Timer1.setPeriod(isr_us);
  interrupts();
}

static void j1_ol_stepPulse()
{
  if (!j1_ol_run) {
    digitalWrite(j1_pul, LOW);
    return;
  }

  digitalWrite(j1_pul, j1_ol_pulse_state);

  // HIGH가 되는 순간을 1 step으로 계산
  if (j1_ol_pulse_state == HIGH) {
    j1_ol_step_count++;

    if (j1_ol_target_steps > 0 &&
        j1_ol_step_count >= j1_ol_target_steps) {
      j1_ol_run = false;
      j1_ol_done = true;
      j1_ol_pulse_state = LOW;
      digitalWrite(j1_pul, LOW);
      return;
    }
  }

  j1_ol_pulse_state = !j1_ol_pulse_state;
}

void j1_ol_move(bool dir, unsigned long pps)
{
  pinMode(j1_pul, OUTPUT);
  pinMode(j1_dir, OUTPUT);
  pinMode(j1_en, OUTPUT);

  // 현재 기존 코드 기준 J1 enable은 HIGH가 동작
  digitalWrite(j1_en, HIGH);

  // 방향은 실제 동작 보고 반대면 HIGH/LOW만 뒤집으면 됨
  digitalWrite(j1_dir, dir ? HIGH : LOW);
  digitalWrite(j1_pul, LOW);
  delayMicroseconds(20);

  noInterrupts();
  j1_run = false;              // 기존 PID용 j1_run 정지
  j1pulseState = LOW;

  j1_ol_run = false;
  j1_ol_pulse_state = LOW;
  j1_ol_target_steps = 0;      // 0이면 연속 구동
  j1_ol_step_count = 0;
  j1_ol_done = false;
  interrupts();

  Timer1.stop();
  Timer1.detachInterrupt();
  Timer1.attachInterrupt(j1_ol_stepPulse);

  j1_ol_set_pps(pps);

  noInterrupts();
  j1_ol_run = true;
  interrupts();

  Timer1.start();
}

void j1_ol_stop(bool disable_after)
{
  noInterrupts();
  j1_ol_run = false;
  j1_ol_pulse_state = LOW;
  j1_ol_target_steps = 0;
  j1_ol_step_count = 0;
  j1_ol_done = false;
  interrupts();

  digitalWrite(j1_pul, LOW);

  Timer1.stop();
  Timer1.detachInterrupt();

  // 기존 엔코더 PID용 ISR 복구
  Timer1.attachInterrupt(j1stepPulse);

  if (disable_after) {
    digitalWrite(j1_en, LOW);
  }
}

bool j1_ol_move_steps(bool dir,
                      long steps,
                      unsigned long pps,
                      unsigned long timeout_ms)
{
  if (steps <= 0) {
    return true;
  }

  pinMode(j1_pul, OUTPUT);
  pinMode(j1_dir, OUTPUT);
  pinMode(j1_en, OUTPUT);

  digitalWrite(j1_en, HIGH);
  digitalWrite(j1_dir, dir ? HIGH : LOW);
  digitalWrite(j1_pul, LOW);
  delayMicroseconds(20);

  noInterrupts();
  j1_run = false;
  j1pulseState = LOW;

  j1_ol_run = false;
  j1_ol_pulse_state = LOW;
  j1_ol_target_steps = steps;
  j1_ol_step_count = 0;
  j1_ol_done = false;
  interrupts();

  Timer1.stop();
  Timer1.detachInterrupt();
  Timer1.attachInterrupt(j1_ol_stepPulse);

  j1_ol_set_pps(pps);

  noInterrupts();
  j1_ol_run = true;
  interrupts();

  Timer1.start();

  unsigned long t0 = millis();

  while (true) {
    if (j1_ol_done) {
      j1_ol_stop(false);
      return true;
    }

    if (millis() - t0 > timeout_ms) {
      j1_ol_stop(false);
      return false;
    }
  }
}

bool j1_ol_move_deg(float delta_joint_deg,
                    unsigned long pps,
                    unsigned long timeout_ms)
{
  long steps = (long)(
    fabs(delta_joint_deg) *
    (float)j1_gear *
    (float)PULSES_PER_REV / 360.0f
  );

  bool dir = (delta_joint_deg >= 0);

  return j1_ol_move_steps(dir, steps, pps, timeout_ms);
}


// -------------------- J3 open-loop state --------------------
static volatile bool j3_ol_run = false;
static volatile bool j3_ol_pulse_state = LOW;
static volatile long j3_ol_target_steps = 0;
static volatile long j3_ol_step_count = 0;
static volatile bool j3_ol_done = false;

static void j3_ol_set_pps(unsigned long pps)
{
  if (pps < 1) pps = 1;

  unsigned long isr_us = 1000000UL / (2UL * pps);

  // 너무 빠른 인터럽트 방지
  if (isr_us < 50) isr_us = 50;

  noInterrupts();
  Timer4.setPeriod(isr_us);
  interrupts();
}

static void j3_ol_stepPulse()
{
  if (!j3_ol_run) {
    digitalWrite(j3_pul, LOW);
    return;
  }

  digitalWrite(j3_pul, j3_ol_pulse_state);

  // HIGH가 되는 순간을 1 step으로 계산
  if (j3_ol_pulse_state == HIGH) {
    j3_ol_step_count++;

    if (j3_ol_target_steps > 0 &&
        j3_ol_step_count >= j3_ol_target_steps) {
      j3_ol_run = false;
      j3_ol_done = true;
      j3_ol_pulse_state = LOW;
      digitalWrite(j3_pul, LOW);
      return;
    }
  }

  j3_ol_pulse_state = !j3_ol_pulse_state;
}

void j3_ol_move(bool dir, unsigned long pps)
{
  pinMode(j3_pul, OUTPUT);
  pinMode(j3_dir, OUTPUT);
  pinMode(j3_en, OUTPUT);

  // 현재 기존 코드 기준 J3 enable은 HIGH가 동작
  digitalWrite(j3_en, HIGH);

  // 방향은 실제 동작 보고 반대면 HIGH/LOW만 뒤집으면 됨
  digitalWrite(j3_dir, dir ? HIGH : LOW);
  digitalWrite(j3_pul, LOW);
  delayMicroseconds(20);

  noInterrupts();
  j3_run = false;              // 기존 PID용 j3_run 정지
  j3pulseState = LOW;

  j3_ol_run = false;
  j3_ol_pulse_state = LOW;
  j3_ol_target_steps = 0;      // 0이면 연속 구동
  j3_ol_step_count = 0;
  j3_ol_done = false;
  interrupts();

  Timer4.stop();
  Timer4.detachInterrupt();
  Timer4.attachInterrupt(j3_ol_stepPulse);

  j3_ol_set_pps(pps);

  noInterrupts();
  j3_ol_run = true;
  interrupts();

  Timer4.start();
}

void j3_ol_stop(bool disable_after)
{
  noInterrupts();
  j3_ol_run = false;
  j3_ol_pulse_state = LOW;
  j3_ol_target_steps = 0;
  j3_ol_step_count = 0;
  j3_ol_done = false;
  interrupts();

  digitalWrite(j3_pul, LOW);

  Timer4.stop();
  Timer4.detachInterrupt();

  // 기존 엔코더 PID용 ISR 복구
  Timer4.attachInterrupt(j3stepPulse);

  if (disable_after) {
    digitalWrite(j3_en, LOW);
  }
}

bool j3_ol_move_steps(bool dir,
                      long steps,
                      unsigned long pps,
                      unsigned long timeout_ms)
{
  if (steps <= 0) {
    return true;
  }

  pinMode(j3_pul, OUTPUT);
  pinMode(j3_dir, OUTPUT);
  pinMode(j3_en, OUTPUT);

  digitalWrite(j3_en, HIGH);
  digitalWrite(j3_dir, dir ? HIGH : LOW);
  digitalWrite(j3_pul, LOW);
  delayMicroseconds(20);

  noInterrupts();
  j3_run = false;
  j3pulseState = LOW;

  j3_ol_run = false;
  j3_ol_pulse_state = LOW;
  j3_ol_target_steps = steps;
  j3_ol_step_count = 0;
  j3_ol_done = false;
  interrupts();

  Timer4.stop();
  Timer4.detachInterrupt();
  Timer4.attachInterrupt(j3_ol_stepPulse);

  j3_ol_set_pps(pps);

  noInterrupts();
  j3_ol_run = true;
  interrupts();

  Timer4.start();

  unsigned long t0 = millis();

  while (true) {
    if (j3_ol_done) {
      j3_ol_stop(false);
      return true;
    }

    if (millis() - t0 > timeout_ms) {
      j3_ol_stop(false);
      return false;
    }
  }
}

bool j3_ol_move_deg(float delta_joint_deg,
                    unsigned long pps,
                    unsigned long timeout_ms)
{
  long steps = (long)(
    fabs(delta_joint_deg) *
    (float)j3_gear *
    (float)PULSES_PER_REV / 360.0f
  );

  bool dir = (delta_joint_deg >= 0);

  return j3_ol_move_steps(dir, steps, pps, timeout_ms);
}
void stopAllMotion(bool disableDrivers)
{
    /*
     * 1. ISR에서 참조하는 모든 동작 상태를 먼저 정지한다.
     *
     * 이 구간에서는 Timer ISR이 중간 상태를 읽지 못하도록
     * 전체 상태를 한 번에 변경한다.
     */
    noInterrupts();

    // ─────────────────────────
    // J1 PID 상태
    // ─────────────────────────
    j1_run = false;
    j1pulseState = LOW;

    // J1 open-loop 상태
    j1_ol_run = false;
    j1_ol_pulse_state = LOW;
    j1_ol_target_steps = 0;
    j1_ol_step_count = 0;
    j1_ol_done = false;

    // ─────────────────────────
    // J2 상태
    // ─────────────────────────
    j2_run = false;
    j2pulseState = LOW;
    j2_target_steps = 0;
    j2_step_count = 0;
    j2_move_done = false;
    j2_endstop_hit = false;

    // ─────────────────────────
    // J3 PID 상태
    // ─────────────────────────
    j3_run = false;
    j3pulseState = LOW;

    // J3 open-loop 상태
    j3_ol_run = false;
    j3_ol_pulse_state = LOW;
    j3_ol_target_steps = 0;
    j3_ol_step_count = 0;
    j3_ol_done = false;

    // ─────────────────────────
    // J4 상태
    // ─────────────────────────
    j4_run = false;
    j4pulseState = LOW;
    j4_step_edges = 0;

    // ─────────────────────────
    // 레일 상태
    // ─────────────────────────
    rail_run = false;
    railPulseState = LOW;

    // 현재 사용되지는 않지만 이전 pulse 상태 변수도 정리
    j1_ps = false;
    j2_ps = false;
    j3_ps = false;
    j4_ps = false;

    interrupts();

    /*
     * 2. 하드웨어 타이머 정지
     *
     * Timer1 = J1 PID/open-loop
     * Timer3 = J2 또는 레일 공유
     * Timer4 = J3 PID/open-loop
     * Timer5 = J4
     */
    Timer1.stop();
    Timer3.stop();
    Timer4.stop();
    Timer5.stop();

    /*
     * 3. 현재 타이머 ISR 소유권 해제
     *
     * ESTOP 후 RESET에서 set_tim()을 호출해
     * 기본 ISR을 다시 연결한다.
     */
    Timer1.detachInterrupt();
    Timer3.detachInterrupt();
    Timer4.detachInterrupt();
    Timer5.detachInterrupt();

    /*
     * 4. 모든 STEP/PULSE 핀을 LOW로 확정
     *
     * run=false만 설정해도 ISR이 LOW로 내리지만,
     * 타이머를 detach했기 때문에 마지막 핀 상태를
     * 명시적으로 LOW로 만들어야 한다.
     */
    digitalWrite(j1_pul, LOW);
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(j3_pul, LOW);
    digitalWrite(j4_pul, LOW);
    digitalWrite(rail_pul, LOW);

    /*
     * 5. PID 누적값 정리
     *
     * ESTOP 전에 누적된 integral/prevError가 남아 있으면
     * RESET 후 첫 PID 이동에서 순간적으로 큰 출력이
     * 만들어질 수 있다.
     *
     * Kp/Ki/Kd/iLimit 및 엔코더 위치는 건드리지 않는다.
     */
    j1_pid.prevError = 0.0f;
    j1_pid.integral = 0.0f;

    j3_pid.prevError = 0.0f;
    j3_pid.integral = 0.0f;

    j4_pid.prevError = 0.0f;
    j4_pid.integral = 0.0f;

    /*
     * 6. 선택적으로 드라이버 Disable
     *
     * false이면 현재 Enable 상태를 유지하므로
     * 모터 토크가 유지된다.
     *
     * SCARA가 트레이를 들고 있거나 수직축이 있는 경우
     * ESTOP에서는 일반적으로 false를 사용한다.
     */
    if (disableDrivers) {
        j1_enable(false);
        j2_enable(false);
        j3_enable(false);
        j4_enable(false);

        // 현재 레일 코드는 HIGH=Enable, LOW=Disable
        digitalWrite(rail_en, LOW);
    }
}