#include "set_motor.h"
#include "encoder.h"
#include "pid.h"


float targetAngle = 7200.0f;

void setup() {
  Serial.begin(115200);
  motor_pin();

  set_int();

  Timer1.initialize(50);
  Timer1.attachInterrupt(stepPulse);
}

void loop() {
  move_j1();  
}