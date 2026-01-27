#include "set_motor.h"
#include "encoder.h"
#include "pid.h"
#include "move.h"


void setup() {
  Serial.begin(115200);
  motor_pin();
  set_tim();
  set_int();
}

void loop() {
  //delay(5000);
  //goXY(0,80);
  //move_j3_wait(90);
  //delay(500);
  //move_j3_wait(90); 
  //move_j4_wait(-30); 
  //delay(100);
  //move_j3_wait(-90);
  //delay(100);
  //move_j3_wait(90);
  //move_j1_wait(200);
  home();
  goXY(0,80);
  //delay(3000);
  //move_j1_wait(200);
  //delay(500);

  //Serial.print("stop_j4="); Serial.println(digitalRead(stop_j4));
  //j4_home_stop_on_switch_safe(true, 1000);
  //delay(500);
  //move_j1_wait(-180);
  //goXY(0,150); // (0,300)
  //goXY(0,250); //(0, 220);
  //goXY(150,0); //(-300,-200)
  //goXY(250,0); //(-250, -130)
  //goXY(0,-150); //(-150, -300)
  //goXY(-150,0); //(-300, 150)
  //goXY(150,150); //(300,170)
  //goXY(-150,150); //(-160,200)
  //goXY(-150,-150); //(-300, -170)
  //goXY(150,-150); //(170, -300)
  //goXY(0,80); //(340, 0)
  //goXY(80,0); //(100,-340)
  //move_j3_wait(7200,5000);
  delay(100000000);
  //move_j2_cm(-5.0f);
  //Serial.println(digitalRead(stop_j3)); // 안누르면 1, 누르면 0 이 나오면 정상
  //delay(200);
}
