#include <Arduino.h>
#include <TimerThree.h>

// ================== 핀 설정 ==================
#define rail_pul  41
#define rail_dir  42
#define rail_en   43
#define stop_rail 44
#define num1      45
#define num2      46
#define num3      47

// EN 논리: 드라이버가 ENA-LOW이면 true, ENA-HIGH이면 false
constexpr bool RAIL_EN_ACTIVE_LOW = false;

// ================== 내부 상태 ==================
static volatile bool  rail_run = false;
static volatile bool  rail_step_state = false;   // 토글 상태
static volatile long  rail_step_edges = 0;       // 상승엣지 카운트 = step 수
static volatile long  rail_target_steps = 0;     // 0이면 무한
static volatile bool  rail_done = false;

// ================== EN 제어 ==================
static inline void rail_enable(bool on)
{
  pinMode(rail_en, OUTPUT);
  digitalWrite(rail_en, on ? (RAIL_EN_ACTIVE_LOW ? LOW : HIGH)
                          : (RAIL_EN_ACTIVE_LOW ? HIGH : LOW));
}

// ================== 엔드스탑 읽기 ==================
// stop_rail: INPUT_PULLUP 가정, 눌리면 LOW
static inline bool rail_stop_pressed()
{
  return (digitalRead(stop_rail) == LOW);
}

// ================== Timer3 ISR ==================
static void rail_isr_step()
{
  if (!rail_run) {
    digitalWrite(rail_pul, LOW);
    rail_step_state = false;
    return;
  }

  digitalWrite(rail_pul, rail_step_state);

  // 상승엣지에서 step 1개 카운트
  if (rail_step_state == HIGH) {
    rail_step_edges++;

    // 목표 step 이동 완료
    if (rail_target_steps > 0 && rail_step_edges >= rail_target_steps) {
      rail_run = false;
      rail_done = true;
      digitalWrite(rail_pul, LOW);
      rail_step_state = false;
      return;
    }

    // 엔드스탑에 닿으면 즉시 정지(홈 방향에서만 쓰는 것이 안전)
    if (rail_stop_pressed()) {
      rail_run = false;
      rail_done = true;
      digitalWrite(rail_pul, LOW);
      rail_step_state = false;
      return;
    }
  }

  rail_step_state = !rail_step_state;
}

// ================== 초기화 ==================
static inline void rail_begin_timer3()
{
  pinMode(rail_pul, OUTPUT);
  pinMode(rail_dir, OUTPUT);
  pinMode(stop_rail, INPUT_PULLUP);

  digitalWrite(rail_pul, LOW);

  // Timer3 기본 세팅
  Timer3.initialize(100);           // 임시
  Timer3.attachInterrupt(rail_isr_step);

  rail_run = false;
  rail_done = false;
  rail_step_edges = 0;
  rail_target_steps = 0;

  // 필요하면 기본 enable
  // rail_enable(true);
}

// ================== 속도 설정 (pps) ==================
static inline void rail_set_speed_pps(unsigned long pps)
{
  if (pps < 1) pps = 1;

  // 토글 기반 => ISR 주기 = 1 / (2*pps)
  unsigned long isr_us = 1000000UL / (2UL * pps);

  // 너무 빠른 값 방지(보드/라이브러리 여유)
  if (isr_us < 20) isr_us = 20;

  noInterrupts();
  Timer3.setPeriod(isr_us);
  interrupts();
}

// ================== 구동 시작(무한) ==================
static inline void rail_start(bool dir_toward_home, unsigned long pps)
{
  rail_enable(true);
  digitalWrite(rail_dir, dir_toward_home ? HIGH : LOW);

  noInterrupts();
  rail_step_edges = 0;
  rail_target_steps = 0; // 무한
  rail_done = false;
  rail_run = true;
  interrupts();

  rail_set_speed_pps(pps);
}

// ================== step 수만큼 이동 ==================
static inline void rail_move_steps(long steps, bool dir, unsigned long pps)
{
  if (steps <= 0) return;

  rail_enable(true);
  digitalWrite(rail_dir, dir ? HIGH : LOW);

  noInterrupts();
  rail_step_edges = 0;
  rail_target_steps = steps;
  rail_done = false;
  rail_run = true;
  interrupts();

  rail_set_speed_pps(pps);
}

// ================== 즉시 정지 ==================
static inline void rail_stop(bool disable_after = false)
{
  noInterrupts();
  rail_run = false;
  interrupts();

  digitalWrite(rail_pul, LOW);
  rail_step_state = false;

  if (disable_after) rail_enable(false);
}

// ================== 동작 완료 대기 ==================
static inline bool rail_wait_done(unsigned long timeout_ms = 0)
{
  unsigned long t0 = millis();
  while (true) {
    noInterrupts();
    bool done = rail_done;
    interrupts();

    if (done) return true;

    if (timeout_ms > 0 && (millis() - t0) > timeout_ms) return false;
    delay(1);
  }
}

// ================== 안전 홈: 스위치까지 접근 + 백오프 ==================
// dir_to_switch: true면 스위치쪽 방향
// backoff_steps: 스위치 눌린 뒤 풀기 위해 뒤로 빼는 step 수
static inline bool rail_home_to_switch(bool dir_to_switch,   //홈가는 함수
                                      unsigned long pps,
                                      long backoff_steps = 200,
                                      unsigned long timeout_ms = 5000)
{
  rail_enable(true);
  pinMode(stop_rail, INPUT_PULLUP);

  // 0) 이미 눌려있으면 backoff부터
  if (rail_stop_pressed()) {
    rail_move_steps(backoff_steps, !dir_to_switch, pps);
    if (!rail_wait_done(1000)) { rail_stop(); return false; }
    delay(50);
  }

  // 1) 스위치까지 접근(엔드스탑 감시는 ISR에서 처리)
  rail_start(dir_to_switch, pps);

  unsigned long t0 = millis();
  while (true) {
    if (rail_stop_pressed()) break;
    if (millis() - t0 > timeout_ms) {
      rail_stop();
      return false;
    }
    delay(1);
  }

  // ISR이 stop 처리하기 전에 여기서도 안전 정지
  rail_stop(false);
  delay(50);

  // 2) backoff (스위치 해제)
  rail_move_steps(backoff_steps, !dir_to_switch, pps);
  if (!rail_wait_done(2000)) { rail_stop(); return false; }

  rail_stop(false);
  return true;
}
void tool(bool on)
{
  if (on)
  {
    digitalWrite(num1, HIGH);
    digitalWrite(num2, HIGH);
    digitalWrite(num3, HIGH);
  }
  else 
  {
    digitalWrite(num1, LOW);
    digitalWrite(num2, LOW);
    digitalWrite(num3, LOW);
  }

}
void setup() {
  Serial.begin(115200);
  rail_begin_timer3();
  pinMode(num1, OUTPUT);
  pinMode(num2, OUTPUT);
  pinMode(num3, OUTPUT);
  // 홈 동작: 스위치 방향이 true라고 가정
  //bool ok = rail_home_to_switch(true, 3000, 300, 6000);
  //Serial.println(ok ? "RAIL HOME OK" : "RAIL HOME FAIL");

  // 5000 step 이동
  //rail_move_steps(5000, true, 2000);
  //rail_wait_done(5000);
  //ail_stop(true);   // 정지 + disable

}

void loop() {

  tool(true);
  delay(5000);
  tool(false);
  delay(5000);
  // put your main code here, to run repeatedly:
  //rail_move_steps(90000, 1, 25000);
  //delay(2000);
}
