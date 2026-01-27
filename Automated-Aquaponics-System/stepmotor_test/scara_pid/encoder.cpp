#include "encoder.h"
#include "set_motor.h"
#include "pid.h"

const float cpr    = 400.0f;
const float en_cnt = cpr * 2.0f;
const int j3_gear = 9;
const int j1_gear = 20;
volatile bool j1_run=false, j2_run=false, j3_run=false, j4_run=false;
static volatile bool j1_ps=false, j2_ps=false, j3_ps=false, j4_ps=false;

encod j1_enc = { j1_A, j1_B, 0 };
encod j3_enc = { j3_A, j3_B, 0 };
encod j4_enc = { j4_A, j4_B, 0 };

volatile bool j1pulseState = false;
volatile bool j3pulseState = false;
volatile bool j4pulseState = false;

volatile long j1pulseInterval = 100;
volatile long j3pulseInterval = 100;
volatile long j4pulseInterval = 100;

// z축 
const unsigned int PULSE_US = 10;   // 펄스폭은 넉넉히
const long PULSES_PER_REV = 3200;  // (가정) 1.8°모터 + 128분주
float J2_LEAD_MM_PER_REV = (8.0f/1.0f);
unsigned long J2_DEFAULT_PPS =500000;   // j2 기본 속도
volatile bool j2_endstop_hit = false;

float encoder_getAngleDeg(const encod* e) {
  return -(e->pos) * 360.0f / en_cnt;
}

void set_tim()
{
  Timer1.initialize(50);
  Timer1.attachInterrupt(j1stepPulse);
  Timer4.initialize(50);
  Timer4.attachInterrupt(j3stepPulse);
  Timer5.initialize(50);
  Timer5.attachInterrupt(j4stepPulse);
}

void set_int()
{
  attachInterrupt(digitalPinToInterrupt(j1_enc.pinA), j1EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j3_enc.pinA), j3EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j4_enc.pinA), j4EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(stop_z), j2EndstopISR, FALLING);
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

bool move_j1(float targetAngle, float tolDeg = 1.0f)
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

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 2500) speed = 2500;

  long interval = 1000000L / (2.0f * speed);
  
  noInterrupts();
  j1pulseInterval = interval;
  Timer1.setPeriod(j1pulseInterval);
  interrupts();

  digitalWrite(j1_dir, (pidOut > 0) ? LOW : HIGH);

  //Serial.print("Angle="); Serial.print(nowAngle);
  //Serial.print(" Error="); Serial.print(error);
  //Serial.print(" speed="); Serial.print(speed);
  //Serial.print(" interval(us)="); Serial.println(interval);

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

void move_j1_wait(float targetAngle,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 8000) //연속동작 가능
{
  unsigned long t0 = millis();
  unsigned long inTolSince = 0;

  while (true) {
    move_j1(targetAngle);                 

    float e = j1_error_deg(targetAngle);
    if (fabs(e) <j1_gear* tolDeg) {
      if (inTolSince == 0) inTolSince = millis();
      if (millis() - inTolSince >= stable_ms) break;
    } else {
      inTolSince = 0;
    }

    if (millis() - t0 > timeout_ms) break;
    delay(5);
  }

  // 정지(필요하면)
  j1_run = false;
  digitalWrite(j1_pul, LOW);
}

static void j1_set_pps(unsigned long pps) {
  if (pps < 1) pps = 1;
  unsigned long isr_us = 1000000UL / (2 * pps);  // 토글이라 2배
  if (isr_us < 50) isr_us = 50;                  // 너무 빠른 값 방지(필요시 조절)
  noInterrupts();
  Timer1.setPeriod(isr_us);
  interrupts();
  j1_run = true;
}
void j1_home_stop_on_switch(bool dir_to_switch, unsigned long pps)
{
  pinMode(stop_j1, INPUT_PULLUP);
  delay(500);

  digitalWrite(j1_dir, dir_to_switch ? HIGH : LOW);
  j1_set_pps(pps);
  j1_run = true;

  // 스위치 눌릴 때까지 계속 구동
  while (digitalRead(stop_j1) != LOW) {  }

  // 눌리면 정지
  j1_run = false;
  digitalWrite(j1_pul, LOW);
}
void enc_reset_j1()
{
  noInterrupts();
  j1_enc.pos = 0;
  interrupts();
}
///////////////////////////////////////////////////////////////////////////////////////////////

//관절 2에 대한 함수들 /////////////////////////////////////////////////////////////////////////
void stepOnce(unsigned long pps){
  unsigned long period = 1000000UL / pps;
  if (period <= PULSE_US + 5) period = PULSE_US + 5;
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(PULSE_US);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(period - PULSE_US);
}
bool move_j2(float deg, unsigned long pps)
{
  j2_endstop_hit = false;

  bool toward_endstop = (deg >= 0); 
  long steps = (long)(fabs(deg) * (float)PULSES_PER_REV / 360.0f);

  digitalWrite(DIR_PIN, (deg >= 0) ? HIGH : LOW);

  for (long i = 0; i < steps; i++) {
    if (toward_endstop && (digitalRead(stop_z) == LOW)) {
      return false; // 엔드스탑 hit
    }
    stepOnce(pps);
  }
  return true;
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

bool move_j3(float targetAngle, float tolDeg = 1.0f)
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

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 4000) speed = 4000;

  long interval = 1000000L / (2.0f * speed);

  noInterrupts();
  j3pulseInterval = interval;
  Timer4.setPeriod(j3pulseInterval);
  interrupts();

  digitalWrite(j3_dir, (pidOut > 0) ? LOW : HIGH);

  Serial.print("Angle="); Serial.print(nowAngle);
  Serial.print(" Error="); Serial.print(error);
  Serial.print(" speed="); Serial.print(speed);
  Serial.print(" interval(us)="); Serial.println(interval);

  if (fabs(error) <= j3_gear*tolDeg) {
    j3_run = false;              
    digitalWrite(j3_pul, LOW);   
    return true;                 
  }
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

void move_j3_wait(float targetAngle,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 8000) //연속동작 가능
{
  unsigned long t0 = millis();
  unsigned long inTolSince = 0;

  while (true) {
    move_j3(targetAngle);                 

    float e = j3_error_deg(targetAngle);
    if (fabs(e) <= j3_gear*tolDeg) {
      if (inTolSince == 0) inTolSince = millis();
      if (millis() - inTolSince >= stable_ms) break;
    } else {
      inTolSince = 0;
    }

    if (millis() - t0 > timeout_ms) break;
    delay(5);
  }

  // 정지(필요하면)
  j3_run = false;
  digitalWrite(j3_pul, LOW);
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
void j3_home_stop_on_switch(bool dir_to_switch, unsigned long pps)
{
  pinMode(stop_j3, INPUT_PULLUP);
  delay(500);

  digitalWrite(j3_dir, dir_to_switch ? HIGH : LOW);
  j3_set_pps(pps);
  j3_run = true;

  // 스위치 눌릴 때까지 계속 구동
  while (digitalRead(stop_j3) != LOW) {  }

  // 눌리면 정지
  
  j3_run = false;
  digitalWrite(j3_pul, LOW);
}
void enc_reset_j3()
{
  noInterrupts();
  j3_enc.pos = 0;
  interrupts();
}
////////////////////////////////////////////////////////////////////////////////////////////

//관절 4에 대한 함수들//////////////////////////////////////////////////////////////////////
volatile long j4_step_edges = 0;

void j4stepPulse() {
  if(!j4_run){
    digitalWrite(j4_pul, LOW);
    return;
  }
  digitalWrite(j4_pul, j4pulseState);
  // 상승엣지(LOW->HIGH)에서만 1 step으로 카운트
  if (j4pulseState == HIGH) j4_step_edges++;
  j4pulseState = !j4pulseState;
}

void j4EncoderA() {
  bool a = digitalRead(j4_enc.pinA);
  bool b = digitalRead(j4_enc.pinB);
  j4_enc.pos += (a == b) ? 1 : -1;
}

bool move_j4(float targetAngle, float tolDeg = 1.0f)
{
  float Angle = 4.5 * targetAngle;

  j4_run = true;            
  
  noInterrupts();
  long Count = j4_enc.pos;
  interrupts();

  encod snap = j4_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = Angle + nowAngle;
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
  //Serial.print(" Error="); Serial.print(error);
  //Serial.print(" speed="); Serial.print(speed);
  //Serial.print(" interval(us)="); Serial.println(interval);
  
  if (fabs(error) <= 4.5*tolDeg) {
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
  return (4.5f * targetAngle) + j4_now_deg();
} 

void move_j4_wait(float targetAngle,
                  float tolDeg = 1.0f,
                  unsigned long stable_ms = 150,
                  unsigned long timeout_ms = 8000) //연속동작 가능
{
  unsigned long t0 = millis();
  unsigned long inTolSince = 0;

  while (true) {
    move_j4(targetAngle);                 

    float e = j4_error_deg(targetAngle);
    if (fabs(e) <= 4.5*tolDeg) {
      if (inTolSince == 0) inTolSince = millis();
      if (millis() - inTolSince >= stable_ms) break;
    } else {
      inTolSince = 0;
    }

    if (millis() - t0 > timeout_ms) break;
    delay(5);
  }

  // 정지(필요하면)
  j4_run = false;
  digitalWrite(j4_pul, LOW);
}
// 필요하면 네 enable 핀에 맞게 수정 (TB6600: ENA-LOW enable인 경우 많음)
static inline void j4_enable(bool on){
  // pinMode(j4_en, OUTPUT);
  // digitalWrite(j4_en, on ? LOW : HIGH);
}

static void j4_set_pps(unsigned long pps) { 
  if (pps < 1) pps = 1; unsigned long isr_us = 1000000UL / (2 * pps); // 토글이라 2배 if (isr_us < 50) isr_us = 50; // 너무 빠른 값 방지(필요시 조절)
  noInterrupts(); 
  Timer5.setPeriod(isr_us); 
  interrupts();
  j4_run = true; 
}

bool j4_home_stop_on_switch_safe(bool dir_to_switch, unsigned long pps)
{
  pinMode(stop_j4, INPUT_PULLUP);
  delay(20);

  j4_enable(true);

  auto setDirToward = [&](bool toward){
    // toward == true면 스위치쪽으로 가는 방향
    // 여기 HIGH/LOW 매핑은 네 하드웨어에 맞게 “한 번만” 정해라
    digitalWrite(j4_dir, toward ? HIGH : LOW);
  };

  // 0) 눌려있으면: 일정 step만 backoff
  if (digitalRead(stop_j4) == LOW) {
    setDirToward(false);               // 스위치 반대 방향
    noInterrupts(); j4_step_edges = 0; interrupts();
    j4_set_pps(pps);
    j4_run = true;

    // 예: 200 step 정도 빼기(기구에 맞게 조절)
    while (j4_step_edges < 200) {
      // timeout (예: 300ms)
      static unsigned long t0 = millis();
      if (millis() - t0 > 300) break;
    }

    j4_run = false;
    digitalWrite(j4_pul, LOW);
    delay(30);
  }

  // 1) 스위치 쪽으로 접근(눌릴 때까지), timeout 필수
  setDirToward(true);
  j4_set_pps(pps);
  j4_run = true;

  unsigned long t0 = millis();
  while (digitalRead(stop_j4) != LOW) {
    if (millis() - t0 > 3000) {  // 3초 내에 안 눌리면 실패
      j4_run = false;
      digitalWrite(j4_pul, LOW);
      j4_enable(false);
      return false;
    }
  }

  // 2) 정지 + (선택) 살짝 backoff해서 스위치 해제 위치 확보
  j4_run = false;
  digitalWrite(j4_pul, LOW);
  delay(30);

  // backoff(예: 100 step)
  setDirToward(false);
  noInterrupts(); j4_step_edges = 0; interrupts();
  j4_set_pps(pps);
  j4_run = true;

  while (j4_step_edges < 100) {
    static unsigned long t1 = millis();
    if (millis() - t1 > 300) break;
  }

  j4_run = false;
  digitalWrite(j4_pul, LOW);

  // 여기서 엔코더 0점 설정
  enc_reset_j4();

  // 발열 줄이려면 disable
  j4_enable(false);
  return true;
}

void enc_reset_j4()
{
  noInterrupts();
  j4_enc.pos = 0;
  interrupts();
}
////////////////////////////////////////////////////////////////////////////////


void enc_reset_all()
{
  noInterrupts();
  j1_enc.pos = 0;
  j3_enc.pos = 0;
  j4_enc.pos = 0;
  interrupts();
}






