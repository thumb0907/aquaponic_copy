#include "utils.h"
#include "step.h"
#include "usb.h"  

void setup()
{
  
  Serial.begin(115200);
  initManipulator();
  usb_begin(Serial);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

  digitalWrite(PUL, HIGH); // 기본 OFF (옵토 끔)

  Serial.println("==== OpenManipulator Started! ====");
}


void loop()
{
  float ang;
  if (usb_poll_angle(ang)) {
    // 여기서 ang(도 단위) 활용
    Serial.print("[RX] angle = ");
    Serial.println(ang, 2);
  }
  /*
  moveHome();
  
  for (int i = 0; i < 2000; i++) stepPulse(dir_state, 500);
  
  dir_state = !dir_state;      // 다음 루프에서 방향 반대
  delay(1000);
  */
}




