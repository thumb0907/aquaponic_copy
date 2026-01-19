#include "pump_motor.h"
#include "water_level.h"
#include "emergency.h"

void setup() {
  Serial.begin(9600);
  pump_pin();
  lev_pin();
  emergency_pin();
}
void loop() {
  lev();
  if (start)
  {
    digitalWrite(led_pin, HIGH);
    digitalWrite(buz_pin, HIGH);
    if (digitalRead(sw_pin)) 
    {
      Serial.println("start");
      sw = true;
      start = true;
      stop = false;
    }
    if ((sw)&&(start)) 
    { 
      digitalWrite(led_pin, LOW);
      if ((wl_mm1 < 150)||(wl_mm1>400))
      {
        pump(off,on,on,on);
        if ((wl_mm2 < 100)&&(wl_mm1 < 150))
        {
          pump(off,off,on,on);    
        }
      } 
      else if ((wl_mm2 < 100)||(wl_mm2>130))
      {
        pump(on,off,on,on);
        if ((wl_mm1>400)&&(wl_mm2>130))
        {
          pump(off,off,on,on);    
        } 
      }
      else pump(on,on,on,on);
    }
  }
  if (stop) pump(off, off, off, off);
}
