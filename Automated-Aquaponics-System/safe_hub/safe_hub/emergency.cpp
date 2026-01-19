#include "emergency.h"  

const byte stop_pin = 3;
const byte start_pin = 2;
const byte sw_pin = 9;
const byte led_pin = 10;
bool stop = false;
bool start = true;
bool sw = false;

void emergency_pin()
{
  pinMode(stop_pin, INPUT);
  pinMode(start_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(stop_pin), stopISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(start_pin), startISR, RISING); 
}
void stopISR() 
{          
  stop = true;   
  start = false;  
  sw = false;
}
void startISR() 
{          
  stop = false;   
  start = true;
}
bool SW()
{
  digitalRead(sw_pin);
}