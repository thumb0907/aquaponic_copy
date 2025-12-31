#include "pump_motor.h"
#include "water_level.h"

const byte exti = 2;
volatile byte state = false;

void setup() {
  pump_pin();
  pinMode(exti, INPUT);
}
void loop() {
  pump(on, on, on, on);
  //lev();
  if (state == true) pump(off, off, off, off);
}

void stop()
{
  state = !state;
}