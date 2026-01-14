#include "utils.h"
#include "step.h"

void setup()
{
  
  Serial.begin(115200);
  initManipulator();
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

  digitalWrite(PUL, HIGH); // 기본 OFF (옵토 끔)

  Serial.println("==== OpenManipulator Started! ====");
}


void loop()
{
  moveHome();
  
  for (int i = 0; i < 2000; i++) stepPulse(dir_state, 100);
  
  dir_state = !dir_state;      // 다음 루프에서 방향 반대
  delay(1000);
}




