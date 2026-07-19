#include "set_motor.h"
#include "encoder.h"
#include "pid.h"
#include "move.h"
#include "path.h"
#include "Serial.h"
#include "servo_config.h"
static const bool SCARA_STANDALONE_TEST = false;
void setup() {
  serialProtocolBegin(115200);

  setServo();
  delay(50);
  motor_pin();
  set_tim();
  set_int();

  setflag();
  updatePrevFlags();

  sendState(SCARA_STATE_IDLE);
}

void loop() {
  if (SCARA_STANDALONE_TEST) {
    runStandaloneMotionTest();
    return;
  }

  serialReceiveTask();

  if (isEstopRequested()) {
    clearEstopRequest();

    pathEmergencyStop();

    setSmf(0);
    sendSmf();
    sendState(SCARA_STATE_ESTOP);
    return;
  }

  if (isResetRequested()) {
    clearResetRequest();

    pathResetState();
    setflag();
    updatePrevFlags();

    sendState(SCARA_STATE_IDLE);
    return;
  }

  pathTask();
}

void runStandaloneMotionTest() {
  //serialReceiveTask();
  //pathTask();


  //move_j2_cm(-1);
  //delay(100000000000);
  //move_sync_13(30, 30, 1000,1000, 1.0, 150, 8000);
 /**/
 //sect3
  //delay(1000000000000000000);
///*
  home();
  urf=0;
  ulf++;
  moveRail_untilStop(false, 4000, stop3_rail);
  if (((urf == 0)&&(ulf == 1))||((urf == 1)&&(ulf == 1)))
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
    
    ulf --;
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
  else if ((urf == 1)&&(ulf == 0))
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
    move_j1_wait(40, 2000);
    delay(200);
    enc_reset_j1();
    
    
    
    
    moveRail(750,0);
    delay(1400);
    stopRail();
    urf --;
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
  move_j3_wait(30+30);
  enc_reset_j3();
  move_j2_cm(-1);
  if (((wlf == 1)&&(wrf == 0))||((wlf==0)&&(wrf==0)))
  {
    enc_reset_j1();
    delay(20);
    move_j1_wait(210,3000);
    delay(200);
    enc_reset_j1();

    move_j2_cm(2);

    enc_reset_j3();
    delay(20);
    move_j3_wait(-20-30);
    delay(200);
    enc_reset_j3();
    j4_move(true, 1600);
    delay(500);
    j4_move(true, 300);
    //moveRail(1420,0);
    delay(2000);
    stopRail();
    j4_stop();
    enc_reset_j3();
    move_j3_wait(-25);
    enc_reset_j3();
    
    //j4_move(true, 50);
    moveRail(1800,0);
    delay(2000);
    stopRail();
    //j4_home();
    delay(50);
    
    enc_reset_j1();
    delay(150);
    move_j1_wait(-5, 1000);
    enc_reset_j1();
    delay(10);    

    move_j2_cm(-14.5 - 5);
    delay(10);
    grip(true); //놓기
    move_j2_cm(17);

    moveRail_untilStop(true, 4000, stop2_rail);

    enc_reset_j1();
    j1_home_stop_on_switch(true, 3200);
    enc_reset_j1();
    delay(10000000000);


    move_j1_wait(-15);
    enc_reset_j1();
    j4_move(true, 500);
    delay(1500);
    j4_stop();
    j4_move(false, 650);
    delay(1500);
    j4_stop();
    move_j2_cm(-1);
    grip(true); 
  }
  else if ((wlf == 0)&&(wrf == 1))
  {
    move_j1_wait(50);
  }
  move_j2_cm(15);
  delay(10);
  moveRail_untilStop(true, 4200, stop3_rail);
  move_j2_cm(-1);
  j1_home_stop_on_switch(true, 4200);
  move_j1_wait(30);
  j2_home_stop_on_switch(true, 4000);
  delay(50);
  j4_move(true, 2500);
  delay(1000);
  j4_stop();  
  j4_home();
  j3_home_stop_on_switch(false, 5000);
  enc_reset_j3();
  delay(10);
  move_j3_wait(30);
  delay(1000000);
//*/
  /*
  Serial.print("A=");
  Serial.print(digitalRead(j1_A));
  Serial.print("  B=");
  Serial.print(digitalRead(j1_B));
  Serial.print("  pos=");
  Serial.println(j1_enc.pos);
  delay(50);
  */
  
  //Serial.println(digitalRead(stop_j4)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(15);
  
  //Serial.println(digitalRead(stop_z)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(5);
}


