#include "step.h"

const int PUL = 5;
const int DIR = 7;
bool dir_state = 0;

void stepPulse(bool dir, int speed) {
  if (dir == 0) // 후진
  {
    digitalWrite(DIR, dir);
    digitalWrite(PUL, LOW);          // ON (옵토 켬)
    delayMicroseconds(speed);
    digitalWrite(PUL, HIGH);         // OFF
    delayMicroseconds(10);          // 속도(작을수록 빠름)
    Serial.println("후진");
  }
  else if(dir == 1)
  {
    digitalWrite(DIR, dir);
    digitalWrite(PUL, LOW);          // ON (옵토 켬)
    delayMicroseconds(10);
    digitalWrite(PUL, HIGH);         // OFF
    delayMicroseconds(speed);          // 속도(작을수록 빠름)
    Serial.println("전진");
  }
}