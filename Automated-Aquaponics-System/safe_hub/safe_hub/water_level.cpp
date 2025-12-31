#include "water_level.h"

void lev_pin()
{
   pinMode(wl, INPUT); 
}
void lev()
{
  analogRead(wl);
}