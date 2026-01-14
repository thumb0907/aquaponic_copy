#include "utils.h"
#include "step.h"

void setup()
{
  Serial.begin(115200);
  step_pin();
  delay(1000);
  initManipulator();
  Serial.println("==== OpenManipulator Started! ====");
}


void loop()
{
  //moveHome();
  static float pps = 300;
  static int dir = +1;

  pps += dir * 50;          // 50ms마다 50pps씩 변화
  if (pps > 3000) { pps = 3000; dir = -1; }
  if (pps < 300)  { pps = 300;  dir = +1; }

  step_setStepsPerSec(pps);
  delay(50);
}




