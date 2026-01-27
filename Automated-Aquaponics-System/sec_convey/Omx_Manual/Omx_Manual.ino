#include "utils.h"
#include "step.h"
#include "usb.h"  

void setup()
{
  
  Serial.begin(115200);
  initManipulator();
  usb_begin(Serial);
  setstep();

  Serial.println("==== OpenManipulator Started! ====");
}


void loop()
{
<<<<<<< HEAD
  /*float ang;
=======
  /*
  float ang;
>>>>>>> 82280afd5b99d0c136265fe565423a1b2d09f277
  if (usb_poll_angle(ang)) {
    Serial.print("[RX] angle = ");
    Serial.println(ang, 2);
<<<<<<< HEAD
  }*/
  
  moveCM(0, 5, 500);
  //delay(2000);
  
  //moveHome();
  
  //for (int i = 0; i < 2000; i++) stepPulse(0, 200);
  
  //dir_state = !dir_state;      // 다음 루프에서 방향 반대
  delay(100000);
  
=======

    // 각도 수신되면 홈으로 이동
    moveHome();
    for (int i = 0; i < 1000; i++) stepPulse(0, 500);
    //Serial.println("[INFO] moveHome() called");
    
  }*/

  //moveHome();
  //for (int i = 0; i < 1000; i++) stepPulse(0, 1000);
  //delay(1000);
  
  /*
  moveHome();

   if (stop_req) {
    STOP_NOW();
    while(1) { delay(10); }  // 테스트용: 완전 정지 유지
  }

  // 예: 이동 루프에서도 stop_req 체크
  for (int i=0; i<50000; i++) {
    if (stop_req) { STOP_NOW(); break; }
    stepPulse(0, 200);
  }
  */
  //readJoint();
  moveHome();
  delay(100000);
>>>>>>> 82280afd5b99d0c136265fe565423a1b2d09f277
}




