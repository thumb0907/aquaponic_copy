#include "encoder.h"
#include "kinematic.h"
#include "move.h"
#include "pid.h"
#include "set_motor.h"
#include "path.h"
#include "Serial.h"

// =========================
// path 내부 상태
// =========================
enum PathState : uint8_t {
  PATH_IDLE,
  PATH_SECT0,
  PATH_SECT1,
  PATH_SECT2,
  PATH_SECT3
};
enum Sect3Step : uint8_t {
  SECT3_STEP_IDLE,
  SECT3_STEP_INIT,
  SECT3_STEP_START_HARVEST,
  SECT3_STEP_HARVEST_RUNNING,
  SECT3_STEP_DONE
};
enum Sect3Phase : uint8_t {
  SECT3_PHASE_LOCKED,       // 선점 불가
  SECT3_PHASE_PREEMPTIBLE   // 선점 가능
};
enum HarvestStep : uint8_t {
  HARVEST_STEP_IDLE,
  HARVEST_STEP_MOVE_TO_CELL,
  HARVEST_STEP_CUT,
  HARVEST_STEP_ADVANCE_INDEX,
  HARVEST_STEP_DONE
};
static PathState currentPath = PATH_IDLE;
static PathState suspendedPath = PATH_IDLE;

static bool sect0_started = false;
static bool sect1_started = false;
static bool sect2_started = false;
static bool sect3_started = false;

static bool step_command_issued = false;
static bool sect3_preemptible = false;

static bool pending_sect1 = false;
static bool pending_sect2 = false;

static Sect3Step sect3_step = SECT3_STEP_IDLE;

int base_x = 80;
int base_y = 0;

static uint16_t active_uv_selector = 0;
static uint8_t active_source = SCARA_SLOT_NONE;
static uint8_t active_destination = SCARA_SLOT_NONE;
// =========================
// 실제 하드웨어 제어 함수 자리
// 지금은 뼈대만
// =========================
static void reportMotionStart() {
  reportMotionStart();
  sendState(SCARA_STATE_MOVING);
}

static void reportMotionIdle() {
  setSmf(0);
  sendSmf();
  sendState(SCARA_STATE_IDLE);
}
static void startScaraMotion(void) {
  home();
}

// =========================
// 섹션 함수
// =========================
void sect0(void){
  if (!sect0_started) {
    sect0_started = true;

    reportMotionStart();
  }

  home();
  delay(500);

  moveRail(3000, 0);   
  delay(900);
  stopRail();

  move_j2_cm(-4.2);

  moveRail_untilStop(true, 4000, stop_rail);

  moveRail(2000, 1);
  delay(2025);
  stopRail();

  delay(300);

  setSmf(0);
  sendSmf();

  setHm(0);
  sendHm();
  reportMotionIdle();

  sect0_started = false;
  currentPath = PATH_IDLE;
}

void sect1(void) {
  if (!sect1_started) {
    sect1_started = true;
    reportMotionStart();
  }
  
  enc_reset_j3();
  move_j3_wait(155+45+5-360+75);
  //delay(100000);
  j1_home_stop_on_switch(false, 3200);
  delay(1300);
  enc_reset_j1();
  delay(10);
  move_j1_wait(45,1500);
  enc_reset_j1();
  delay(500);

  j4_move(true, 500);
  delay(140);
  j4_stop();
  grip(true);//그립 펴고
  //delay(100);
  delay(300);
  move_j2_cm(-12.5); // 내려가서
  grip(false); //잡고
  //delay(100);
  delay(300);
  //delay(300);
  move_j2_cm(1); // 살짝 올라가서
  enc_reset_j1();
  delay(100);
  move_j1_wait(-20, 1000); // 뒤로 좀 빼고
  delay(200);
  enc_reset_j1();
  move_j2_cm(10); // 올라가고
  //move_j1_wait(-10);
  //move_j2_cm(3);

  setCrf(0); // 직교로봇 리셋
  sendCrf();
  j4_move(false, 2000);
  delay(140);
  j4_stop();
  move_j2_cm(3);
  enc_reset_j3();
  delay(200);
  move_j3_wait(30,2000);
  enc_reset_j3();
  delay(20);
  //uv++;
  j1_home_stop_on_switch(true, 1200);
  delay(500);
  enc_reset_j1();
  
  
  if (uv == 1)
  {
    moveRail_untilStop(false, 4500, stop3_rail);
    moveRail(1500,1);   
    delay(1000);
    stopRail();
    move_j2_cm(-1.5);
    
    j4_move(false, 830);
    delay(2000);
    j4_stop();

    enc_reset_j3();
    move_j3_wait(-20);
    enc_reset_j3();
    
    enc_reset_j1();
    move_j1_wait(45, 2500);
    enc_reset_j1();
    
    
    moveRail(2600,1);   
    delay(2000);
    
    moveRail(1200,0);
    delay(1400);
    stopRail();
    move_j2_cm(-3.7);
    grip(true);
    delay(10);
    move_j2_cm(3.7);
    j1_home_stop_on_switch(true, 3500);
    delay(500);
    uv++;
  }
  else if(uv == 0)
  {
    moveRail_untilStop(false, 4500, stop3_rail);
    moveRail(2000,0);   
    delay(1000);
    stopRail();
    move_j2_cm(-1.5);
    enc_reset_j3();
    move_j3_wait(-20);
    delay(200);
    enc_reset_j3();
    enc_reset_j1();
    move_j1_wait(45, 2000);
    delay(80);
    enc_reset_j1();
    j4_move(false, 800);
    moveRail(650,0);   
    delay(2000);
    j4_stop();
    //moveRail(700,1);
    //delay(2000);
    stopRail();
    move_j2_cm(-3.7);
    grip(true);
    delay(10);
    j4_move(true, 1500);
    delay(300);
    j4_move(false, 1500);
    delay(300);
    j4_move(true, 1500);
    delay(300);
    j4_move(false, 1500);
    delay(300);
    j4_stop();
    move_j2_cm(3.7);
    j1_home_stop_on_switch(true, 3500);
    enc_reset_j1();
    delay(500);
    uv++;
  }
  
  // 정상 완료 신호를 먼저 보내 Master가 active job을 확정 종료하게 한다.
  setSsf(0);
  sendSsf();
  reportMotionIdle();
  setSmf(0); // 스카라 구동 끝
  sendSmf();
  
  sect1_started = false;
  currentPath = PATH_IDLE;
  
}

void sect2(void) {
  if (!sect2_started) {
    sect2_started = true;
    reportMotionStart();
    startScaraMotion();
  }
  moveRail(3000,0);   
  delay(2000);
  stopRail();
  move_j1_wait(30);
  enc_reset_j3();
  move_j3_wait(30);

  // 발아실 -> 수경재배실 적재가 정상 완료된 뒤에만 완료 회신
  setS2f(0);
  sendS2f();
  reportMotionIdle();

  setSmf(0);
  sendSmf();

  sect2_started = false;
  currentPath = PATH_IDLE;
}

void sect3(void) {
  switch (sect3_step) {
    case SECT3_STEP_IDLE:
      sect3_started = true;
      reportMotionStart();
      step_command_issued = false;
      sect3_step = SECT3_STEP_INIT;
      break;

    case SECT3_STEP_INIT:
      if (!step_command_issued) {
        //sect3_init();
        step_command_issued = true;
      }

      // 예: if (is_j1_done())
      if (true) {
        step_command_issued = false;
        sect3_step = SECT3_STEP_START_HARVEST;
      }
      break;

    case SECT3_STEP_START_HARVEST:
      if (!step_command_issued) {
        // STM2 FF=1은 S3F=0을 받은 Master가 전송한다.
        // SCARA가 FF를 직접 보내면 시작 주체가 둘이 되므로 여기서는 보내지 않는다.
        step_command_issued = true;
      }

      // 예: if (is_j2_done())
      if (true) {
        step_command_issued = false;
        sect3_step = SECT3_STEP_HARVEST_RUNNING;
      }
      break;


    case SECT3_STEP_DONE:
      // 진동부를 거쳐 C2에 트레이를 내려놓은 뒤에만 완료 회신
      setS3f(0);
      sendS3f();
      reportMotionIdle();
      setSmf(0);
      sendSmf();

      sect3_started = false;
      sect3_step = SECT3_STEP_IDLE;
      currentPath = PATH_IDLE;
      break;
  }
}
// =========================
// 전체 동작 판단
// =========================
void pathTask(void) {
  uint8_t hm  = getHm();
  uint8_t ssf = getSsf();
  uint8_t s2f = getS2f();
  uint8_t s3f = getS3f();

  uint8_t prev_hm  = getPrevHm();
  uint8_t prev_ssf = getPrevSsf();
  uint8_t prev_s2f = getPrevS2f();
  uint8_t prev_s3f = getPrevS3f();
  // -------------------------
  // ssf 상승에지: 0 -> 1
  // 스카라 시작 섹션 진입
  // -------------------------
  if (prev_hm == 0 && hm == 1) {
  if (currentPath == PATH_IDLE) {
      currentPath = PATH_SECT0;
    }
  }

  if (prev_ssf == 0 && ssf == 1) {
    if (currentPath == PATH_IDLE) {
      active_uv_selector = getUv();
      currentPath = PATH_SECT1;
    }
  }

  // -------------------------
  // ssf 하강에지: 1 -> 0
  // 필요 시 강제 정지
  // -------------------------
  if (prev_ssf == 1 && ssf == 0) {
    //stopScaraMotion();
    sect1_started = false;
    setSmf(0);
    sendSmf();
    if (currentPath == PATH_SECT1) {
      currentPath = PATH_IDLE;
    }
  }


  // S2F 상승에지: 발아실 -> 수경재배실
  if (prev_s2f == 0 && s2f == 1) {
    if (currentPath == PATH_IDLE) {
      active_source = getScaraSrc();
      active_destination = getScaraDst();

      if (
        active_source != SCARA_SLOT_NONE &&
        active_destination != SCARA_SLOT_NONE
      ) {
        currentPath = PATH_SECT2;
      }
    }
  }

  // S3F 상승에지: 수경재배실 -> 진동부 -> C2
  if (prev_s3f == 0 && s3f == 1) {
    if (currentPath == PATH_IDLE) {
      active_source = getScaraSrc();
      active_destination = getScaraDst();

      if (
        active_source != SCARA_SLOT_NONE &&
        active_destination == SCARA_SLOT_NONE
      ) {
        currentPath = PATH_SECT3;
      }
    }
  }

  // -------------------------
  // 현재 섹션 수행
  // -------------------------
switch (currentPath) {
  case PATH_IDLE:
    break;

  case PATH_SECT0:
    sect0();
    break;

  case PATH_SECT1:
    sect1();
    break;

  case PATH_SECT2:
    sect2();
    break;

  case PATH_SECT3:
    sect3();
    break;

  default:
    currentPath = PATH_IDLE;
    break;
}

  // 마지막에 이전값 갱신
  updatePrevFlags();
}
