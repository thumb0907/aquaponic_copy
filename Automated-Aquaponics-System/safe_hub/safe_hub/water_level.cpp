#include "water_level.h"
#include "emergency.h"
#include "pump_motor.h"

const int wl_pin[2] = {A0,A1};
const uint8_t buz_pin = 8;

int rawval1;
int rawval2;
float wl_mm1;
float wl_mm2;

void lev_pin()
{
  pinMode(buz_pin, OUTPUT);
}
void lev()
{
  rawval1 = analogRead(wl_pin[0]);
  rawval2 = analogRead(wl_pin[1]);
  wl_mm1 = 42.0 + ((rawval1 - 722.0) * 288.0 / (1022.0 - 722.0));
  wl_mm2 = 42.0 + ((rawval2 - 722.0) * 288.0 / (1022.0 - 722.0));  
  Serial.print("Level1(mm): ");
  Serial.println(wl_mm1, 1);
  Serial.print("Level2(mm): ");
  Serial.println(wl_mm2, 1);
  delay(200);  // 1초 주기
}