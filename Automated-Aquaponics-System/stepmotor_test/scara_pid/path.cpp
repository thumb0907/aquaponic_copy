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

static PathState currentPath = PATH_IDLE;

static uint16_t activeUv = 0;
static uint8_t activeSource = SCARA_SLOT_NONE;
static uint8_t activeDestination = SCARA_SLOT_NONE;

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
// =========================
// 실제 하드웨어 제어 함수 자리
// 지금은 뼈대만
// =========================
static void reportMotionStart() {
  setSmf(1);
  sendSmf();
  sendState(SCARA_STATE_MOVING);
}
static void reportMotionIdle() {
  setSmf(0);
  sendSmf();
  sendState(SCARA_STATE_IDLE);
}

static void finishHome() {
  reportMotionIdle();

  setHmf(0);
  sendHmf();       // 완료 통지는 마지막
}

static void finishSect1() {
  reportMotionIdle();

  setSsf(0);
  sendSsf();
}

static void finishSect2() {
  reportMotionIdle();

  setS2f(0);
  sendS2f();
}

static void finishSect3() {
  reportMotionIdle();

  setS3f(0);
  sendS3f();
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


  finishHome();

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

  //setCrf(0); // 직교로봇 리셋
  //sendCrf();
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
  
   
  if (activeUv == 1) // 왼쪽
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
    //uv++;
  }
  else if(activeUv == 0)
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
    //uv++;
  }
  
  finishSect1();

  sect1_started = false;
  currentPath = PATH_IDLE;
}

void sect2(void) {
  if (!sect2_started) {
    sect2_started = true;
    reportMotionStart();
    //startScaraMotion();
    home();
  }
    
  
  //urf=0;
  //ulf++;
  moveRail_untilStop(false, 4000, stop3_rail);
  if (activeSource == SCARA_SLOT_LEFT)
  {
    move_j2_cm(-5);
    j1_home_stop_on_switch(false, 1200);
    delay(500);
    enc_reset_j1();
    
    moveRail_untilStop(false, 4500, stop3_rail);
    moveRail(2000,0);   
    delay(1000);
    stopRail();
    //move_j2_cm(-1.5);
    
    enc_reset_j3();
    move_j3_wait(-50);
    delay(200);
    enc_reset_j3();

    enc_reset_j1();
    delay(15);
    move_j1_wait(40, 1500);
    delay(200);
    enc_reset_j1();
    
    j4_move(false, 1100);
    moveRail(100,0);   
    delay(2000);
    j4_stop();
    //moveRail(700,1);
    //delay(2000);
    stopRail();
    
    //ulf --;
    grip(true);
  //delay(100);
  move_j2_cm(-4.2);
  grip(false); //발아실의 트레이를 집음
  delay(50);
  move_j2_cm(6.2);
  delay(30);
  moveRail_untilStop(false, 4000, stop2_rail); 
  //enc_reset_j3();
  j1_home_stop_on_switch(true, 2200);
  delay(500);
  //move_j1_wait(130);
  enc_reset_j1();
  
  //j4_move(false, 1400);
  //delay(800);
  j4_stop();
  }
  else if (activeSource == SCARA_SLOT_RIGHT)
  {
    j1_home_stop_on_switch(false, 1200);
    delay(500);
    enc_reset_j1();
    moveRail(1500,1);   
    delay(1000);
    stopRail();
    move_j2_cm(-5);
    
    j4_move(false, 1100);
    moveRail(2600,1);   
    //delay(2000);
    delay(2000);
    j4_stop();
    stopRail();

    enc_reset_j3();
    move_j3_wait(-50);
    enc_reset_j3();
    
    enc_reset_j1();
    delay(20);
    move_j1_wait(40, 2000);
    delay(200);
    enc_reset_j1();
    
    
    
    
    moveRail(750,0);
    delay(1400);
    stopRail();
    //urf --;
    grip(true);
  //delay(100);
    move_j2_cm(-4.2);
    grip(false); //발아실의 트레이를 집음
    delay(50);
    move_j2_cm(6.2);
    delay(30);
    enc_reset_j3();
    
    
    j1_home_stop_on_switch(true, 2200);
    delay(500);
    //move_j1_wait(130);
    enc_reset_j1();
    moveRail_untilStop(false, 4000, stop2_rail); 
  }
  
  j4_move(true, 800);
  delay(800);
  j4_stop();
  //j4_move(false, 800);
  delay(200);
  enc_reset_j3();
  move_j3_wait(30+20);
  delay(200);
  enc_reset_j3();
  move_j2_cm(-1);
  //wlf == 0;
  //wrf = 0;
  if (activeDestination == SCARA_SLOT_RIGHT)
  {
    enc_reset_j1();
    delay(20);
    move_j1_wait(210,3000);
    delay(200);
    enc_reset_j1();

    move_j2_cm(2);

    enc_reset_j3();
    delay(20);
    move_j3_wait(-20-40);
    delay(200);
    enc_reset_j3();
    j4_move(true, 1600);
    delay(500);
    j4_move(true, 500);
    //moveRail(1420,0);
    delay(2000);
    stopRail();
    j4_stop();
    enc_reset_j3();
    move_j3_wait(-25-10);
    delay(150);
    enc_reset_j3();
    
    //j4_move(true, 50);
    moveRail(1850,0);
    delay(2000);
    stopRail();
    //j4_home();
    delay(50);
    
    enc_reset_j1();
    delay(150);
    //move_j1_wait(-5, 1000);
    enc_reset_j1();
    delay(10);    

    move_j2_cm(-14.5 - 3);
    delay(10);
    grip(true); //놓기
    move_j2_cm(17);

    moveRail_untilStop(true, 4000, stop3_rail);

    enc_reset_j1();
    j1_home_stop_on_switch(true, 3200);
    enc_reset_j1();
    //delay(10000000000);
    //wrf ++;
  }

  else if (activeDestination == SCARA_SLOT_LEFT)
  {
    enc_reset_j1();
    delay(20);
    move_j1_wait(105,3000);
    enc_reset_j1();
    enc_reset_j3();
    move_j3_wait(-55);
    delay(200);
    enc_reset_j3();
    moveRail(1900,0);
    delay(2000);
    stopRail();
    

    enc_reset_j3();
    //move_j3_wait(-10);
    delay(200);
    enc_reset_j3();

    j4_move(false, 520);
    delay(400);
    j4_stop();

    enc_reset_j1();
    delay(20);
    move_j1_wait(7,3000);
    enc_reset_j1();
    
    //delay(200000000000);

    move_j2_cm(-14.5 - 3);
    delay(10);
    grip(true); //놓기
    move_j2_cm(17);

    moveRail_untilStop(true, 4000, stop3_rail);

    enc_reset_j1();
    j1_home_stop_on_switch(true, 3200);
    enc_reset_j1();

    //delay(10000000000);
    //wlf ++;
  }


  finishSect2();

  sect2_started = false;
  currentPath = PATH_IDLE;
}

void sect3(void) {
 if (!sect3_started) {
    sect3_started = true;
    //startScaraMotion();
    reportMotionStart();
    home();
  } 

  moveRail_untilStop(false, 4400, stop2_rail); 
  delay(100);
  j1_home_stop_on_switch(false, 1200);
  enc_reset_j1();
  delay(1000);
  move_j2_cm(-4.5);

  if(activeSource == SCARA_SLOT_RIGHT)
  {
    enc_reset_j1();
    delay(20);
    move_j1_wait(205,2500);
    delay(100);
    enc_reset_j1();
    //j1_home_stop_on_switch(true, 3200);
    //delay(500);
    enc_reset_j1();
    delay(15);
    //move_j1_wait(-7,500);
    //delay(200);
    enc_reset_j1();

    enc_reset_j3();
    delay(15);
    move_j3_wait(-25-10-20-40);
    delay(200);
    enc_reset_j3();

    j4_move(false, 1620);
    moveRail(1230,0);
    delay(2000);
    j4_stop();
    stopRail();

    

    grip(true);
    move_j2_cm(-14.5 - 2.5);
    delay(10);
    grip(false); //놓기
    move_j2_cm(17);

    moveRail_untilStop(true, 4000, stop3_rail);
    moveRail(2600,1);
    delay(2000);
    //j4_stop();
    stopRail();
    delay(200);

    j4_move(false, 600);
    delay(500);
    j4_stop();

    enc_reset_j1();
    delay(35);
    move_j1_wait(20,500);
    delay(500);
    enc_reset_j1();

    //wrf --;  
  }

  else if(activeSource == SCARA_SLOT_LEFT)
  {
    enc_reset_j1();
    delay(20);
    move_j1_wait(113,2500);
    delay(100);
    enc_reset_j1();

    enc_reset_j3();
    delay(20);
    move_j3_wait(-55);
    delay(200);
    enc_reset_j3();
    moveRail(1900,0);
    delay(2000);
    stopRail();
    
    j4_move(false, 3020);
    delay(600);
    j4_stop();

    enc_reset_j1();
    delay(20);
    //move_j1_wait(5,300);
    delay(200);
    enc_reset_j1();

    grip(true);
    move_j2_cm(-15 - 2);
    delay(10);
    grip(false); //잡기
    move_j2_cm(17);

    moveRail_untilStop(true, 4000, stop3_rail);

    //j4_move(false, 3020);
    //delay(600);
    //j4_stop();

    

    enc_reset_j1();
    delay(30);
    move_j1_wait(55,1200);
    delay(200);
    enc_reset_j1();
    
    move_j2_cm(3.5);
    delay(200);

    enc_reset_j1();
    delay(30);
    move_j1_wait(75,1200);
    delay(200);
    enc_reset_j1();

    j4_move(false, 3020);
    delay(850);
    j4_stop();
    delay(500);
    //delay(100);
    moveRail(1100,1);
    delay(2000);
    stopRail();
    move_j2_cm(-3.5);

    //wlf --;
  } 
  
  move_j2_cm(-13.5);
  delay(10);
  grip(true); //놓기
  //move_j2_cm(6.5);

  delay(5000);

  //move_j2_cm(-6.5);
  //delay(10);
  grip(false); //잡기
  move_j2_cm(10.5);

  moveRail_untilStop(true, 2000, stop_rail);

  moveRail(2000, 1); 
  delay(800);
  stopRail();

  move_j2_cm(-8.5);
  grip(true);
  move_j2_cm(10.5);

  moveRail(4000, 0); 
  delay(1000);
  stopRail();

  j1_home_stop_on_switch(true, 3200);

  finishSect3();

  sect3_started = false;
  currentPath = PATH_IDLE;
}
// =========================
// 전체 동작 판단
// =========================
void pathTask() {
  const uint8_t hmf = getHmf();
  const uint8_t ssf = getSsf();
  const uint8_t s2f = getS2f();
  const uint8_t s3f = getS3f();

  const uint8_t prevHmf = getPrevHmf();
  const uint8_t prevSsf = getPrevSsf();
  const uint8_t prevS2f = getPrevS2f();
  const uint8_t prevS3f = getPrevS3f();

  if (currentPath == PATH_IDLE) {
    if (prevHmf == 0 && hmf == 1) {
      currentPath = PATH_SECT0;
    }
    else if (prevSsf == 0 && ssf == 1) {
      activeUv = getUv();
      currentPath = PATH_SECT1;
    }
    else if (prevS2f == 0 && s2f == 1) {
      activeSource = getScaraSrc();
      activeDestination = getScaraDst();

      if (
        activeSource != SCARA_SLOT_NONE &&
        activeDestination != SCARA_SLOT_NONE
      ) {
        currentPath = PATH_SECT2;
      }
    }
    else if (prevS3f == 0 && s3f == 1) {
      activeSource = getScaraSrc();
      activeDestination = getScaraDst();

      if (
        activeSource != SCARA_SLOT_NONE &&
        activeDestination == SCARA_SLOT_NONE
      ) {
        currentPath = PATH_SECT3;
      }
    }
  }

  switch (currentPath) {
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

    case PATH_IDLE:
    default:
      break;
  }

  updatePrevFlags();
}