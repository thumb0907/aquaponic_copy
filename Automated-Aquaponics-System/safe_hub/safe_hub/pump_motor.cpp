#include "pump_motor.h"

const uint8_t rel_pin[4] = {4, 5, 6, 7};

void pump_pin()                                          // 펌프모터 핀 설정
{
  for (int i=0; i<4; i++)
  {
    pinMode(rel_pin[i], OUTPUT);
  }
}

void pump(bool a, bool b, bool c, bool d)               //펌프모터 구동 함수
{
  digitalWrite(rel_pin[0], a ? HIGH : LOW);
  digitalWrite(rel_pin[1], b ? HIGH : LOW);
  digitalWrite(rel_pin[2], c ? HIGH : LOW);
  digitalWrite(rel_pin[3], d ? HIGH : LOW);
}
