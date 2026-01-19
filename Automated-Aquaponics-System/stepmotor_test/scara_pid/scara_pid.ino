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
  
  
}