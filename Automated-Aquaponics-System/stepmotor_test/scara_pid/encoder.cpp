#include "encoder.h"
#include "set_motor.h"
#include "pid.h"

const float cpr    = 400.0f;
const float en_cnt = cpr * 2.0f;
volatile bool j1_run=false, j2_run=false, j3_run=false, j4_run=false;
static volatile bool j1_ps=false, j2_ps=false, j3_ps=false, j4_ps=false;

const unsigned int PULSE_US = 25;   // 펄스폭은 넉넉히
const long PULSES_PER_REV = 25600;  // (가정) 1.8°모터 + 128분주

encod j1_enc = { j1_A, j1_B, 0 };
//encod j2_enc = { j2_A, j2_B, 0 };
encod j3_enc = { j3_A, j3_B, 0 };
encod j4_enc = { j4_A, j4_B, 0 };

volatile bool j1pulseState = false;
//volatile bool j2pulseState = false;
volatile bool j3pulseState = false;
volatile bool j4pulseState = false;

volatile long j1pulseInterval = 100;
//volatile long j2pulseInterval = 100;
volatile long j3pulseInterval = 100;
volatile long j4pulseInterval = 100;

float encoder_getAngleDeg(const encod* e) {
  return -(e->pos) * 360.0f / en_cnt;
}

void set_tim()
{
  Timer1.initialize(50);
  Timer1.attachInterrupt(j1stepPulse);
 // Timer3.initialize(50);
  //Timer3.attachInterrupt(j2stepPulse);
  Timer4.initialize(50);
  Timer4.attachInterrupt(j3stepPulse);
  Timer5.initialize(50);
  Timer5.attachInterrupt(j4stepPulse);
}

void set_int()
{
  attachInterrupt(digitalPinToInterrupt(j1_enc.pinA), j1EncoderA, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(j2_enc.pinA), j2EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j3_enc.pinA), j3EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j4_enc.pinA), j4EncoderA, CHANGE);
}

void j1stepPulse() {
  if(!j1_run){ digitalWrite(j1_pul, LOW); return; }
  digitalWrite(j1_pul, j1pulseState);
  j1pulseState = !j1pulseState;
}
/*
void j2stepPulse() {
  if(!j2_run){ digitalWrite(j2_pul, LOW); return; }
  digitalWrite(j2_pul, j2pulseState);
  j2pulseState = !j2pulseState;
}
*/
void j3stepPulse() {
  if(!j3_run){ digitalWrite(j3_pul, LOW); return; }
  digitalWrite(j3_pul, j3pulseState);
  j3pulseState = !j3pulseState;
}
void j4stepPulse() {
  if(!j4_run){ digitalWrite(j4_pul, LOW); return; }
  digitalWrite(j4_pul, j4pulseState);
  j4pulseState = !j4pulseState;
}


void j1EncoderA() {
  bool a = digitalRead(j1_enc.pinA);
  bool b = digitalRead(j1_enc.pinB);
  j1_enc.pos += (a == b) ? 1 : -1;
}

/*
void j2EncoderA() {
  bool a = digitalRead(j2_enc.pinA);
  bool b = digitalRead(j2_enc.pinB);
  j2_enc.pos += (a == b) ? 1 : -1;
}
*/

void j3EncoderA() {
  bool a = digitalRead(j3_enc.pinA);
  bool b = digitalRead(j3_enc.pinB);
  j3_enc.pos += (a == b) ? 1 : -1;
}
void j4EncoderA() {
  bool a = digitalRead(j4_enc.pinA);
  bool b = digitalRead(j4_enc.pinB);
  j4_enc.pos += (a == b) ? 1 : -1;
}

void move_j1(float targetAngle)
{
  j1_run = true;              
  noInterrupts();
  long Count = j1_enc.pos;
  interrupts();

  encod snap = j1_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = targetAngle - nowAngle;
  float pidOut = pid_update(&j1_pid, error);

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 5000) speed = 5000;

  long interval = 1000000L / speed;

  noInterrupts();
  j1pulseInterval = interval;
  Timer1.setPeriod(j1pulseInterval);
  interrupts();

  digitalWrite(j1_dir, (pidOut > 0) ? LOW : HIGH);

  Serial.print("Angle="); Serial.print(nowAngle);
  Serial.print(" Error="); Serial.print(error);
  Serial.print(" speed="); Serial.print(speed);
  Serial.print(" interval(us)="); Serial.println(interval);
}
/*
void move_j2(float targetAngle)
{
  j2_run = true;            
  noInterrupts();
  long Count = j2_enc.pos;
  interrupts();

  encod snap = j2_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = targetAngle - nowAngle;
  float pidOut = pid_update(&j2_pid, error);

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 5000) speed = 5000;

  long interval = 1000000L / speed;

  noInterrupts();
  j2pulseInterval = interval;
  Timer3.setPeriod(j2pulseInterval);
  interrupts();

  digitalWrite(j2_dir, (pidOut > 0) ? LOW : HIGH);

  Serial.print("Angle="); Serial.print(nowAngle);
  Serial.print(" Error="); Serial.print(error);
  Serial.print(" speed="); Serial.print(speed);
  Serial.print(" interval(us)="); Serial.println(interval);
}
*/
void move_j3(float targetAngle)
{
  j3_run = true;            
  noInterrupts();
  long Count = j3_enc.pos;
  interrupts();

  encod snap = j3_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = targetAngle - nowAngle;
  float pidOut = pid_update(&j3_pid, error);

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 5000) speed = 5000;

  long interval = 1000000L / speed;

  noInterrupts();
  j3pulseInterval = interval;
  Timer4.setPeriod(j3pulseInterval);
  interrupts();

  digitalWrite(j3_dir, (pidOut > 0) ? LOW : HIGH);

  Serial.print("Angle="); Serial.print(nowAngle);
  Serial.print(" Error="); Serial.print(error);
  Serial.print(" speed="); Serial.print(speed);
  Serial.print(" interval(us)="); Serial.println(interval);
}

void move_j4(float targetAngle)
{
  j4_run = true;            
  noInterrupts();
  long Count = j4_enc.pos;
  interrupts();

  encod snap = j4_enc;
  snap.pos = Count;
  float nowAngle = encoder_getAngleDeg(&snap);

  float error  = targetAngle + nowAngle;
  float pidOut = pid_update(&j4_pid, error);

  float speed = fabs(pidOut);
  if (speed < 1) speed = 1;
  if (speed > 5000) speed = 5000;

  long interval = 1000000L / speed;

  noInterrupts();
  j4pulseInterval = interval;
  Timer5.setPeriod(j4pulseInterval);
  interrupts();

  digitalWrite(j4_dir, (pidOut > 0) ? LOW : HIGH);

  Serial.print("Angle="); Serial.print(nowAngle);
  Serial.print(" Error="); Serial.print(error);
  Serial.print(" speed="); Serial.print(speed);
  Serial.print(" interval(us)="); Serial.println(interval);
}

void stepOnce(unsigned long pps){
  unsigned long period = 1000000UL / pps;
  if (period <= PULSE_US + 5) period = PULSE_US + 5;
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(PULSE_US);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(period - PULSE_US);
}


void move_j2(float deg, unsigned long pps){
  long pulses = (long)(fabs(deg) * (float)PULSES_PER_REV / 360.0f);
  digitalWrite(DIR_PIN, (deg >= 0) ? LOW : HIGH);
  delayMicroseconds(10);

  for(long i=0;i<pulses;i++){
    if (digitalRead(ALM_PIN) == LOW) { // 알람 발생(극성은 배선에 따라 반대일 수 있음)
      break;
    }
    stepOnce(pps);
  }

  // 펄스 송신이 끝나면 “도착(PEND)”까지 대기
  unsigned long t0 = millis();
  while(millis() - t0 < 3000){  // 3초 타임아웃 예시
    if (digitalRead(PEND_PIN) == LOW) break; // 도착(극성은 환경에 따라 반대일 수 있음)
    if (digitalRead(ALM_PIN) == LOW) break;
  }
}
