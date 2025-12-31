#include "utils.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);
  initManipulator();
  Serial.println("==== OpenManipulator Started! ====");
}


void loop()
{
  moveHome();
}




