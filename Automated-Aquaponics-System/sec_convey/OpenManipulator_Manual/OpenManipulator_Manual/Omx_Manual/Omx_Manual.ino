#include <open_manipulator_libs.h>
#include "utils.h"

String cmd = "";

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println("==== OpenManipulator CLI Pendant ====");
  initManipulator();
  Serial.println("Type 'help' for command list.\n");
}

void loop()
{
  omx.processOpenManipulator(millis() / 1000.0);

  if (Serial.available())
  {
    cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "help")
    {
      Serial.println("===== Command List =====");
      Serial.println("readJoint                 - Show joint angles");
      Serial.println("readTCP                - Show TCP position");
      Serial.println("moveJointAbs j1 j2 j3 j4     - Move to abs joint pos (rad)");
      Serial.println("moveJointRel dj1 dj2 dj3 dj4 - Relative joint move");
      Serial.println("moveTCPAbs x y z             - Move TCP abs pos (m)");
      Serial.println("moveTCPRel dx dy dz       - Move TCP relative (m)");
      Serial.println("toolopen / toolclose   - Gripper control");
      Serial.println("moveHome                   - Move all joints to 0 rad");
      Serial.println("keephori               - Ensure that the TCP remains horizontal.");
      Serial.println("pitch val_deg           - Set gripper pitch angle (deg)\n");
    }

    else if (cmd == "readJoint") readJoint();
    else if (cmd == "readTCP") readTCP();

    else if (cmd.startsWith("moveJointAbs"))
    {
      float j1, j2, j3, j4;
      if (sscanf(cmd.c_str(), "moveJointAbs %f %f %f %f", &j1, &j2, &j3, &j4) == 4)
        moveJointAbs(j1, j2, j3, j4);
      else Serial.println("Usage: moveJointAbs j1 j2 j3 j4");
    }

    else if (cmd.startsWith("moveJointRel"))
    {
      float dj1, dj2, dj3, dj4;
      if (sscanf(cmd.c_str(), "moveJointRel %f %f %f %f", &dj1, &dj2, &dj3, &dj4) == 4)
        moveJointRel(dj1, dj2, dj3, dj4);
      else Serial.println("Usage: moveJointRel dj1 dj2 dj3 dj4");
    }

    else if (cmd.startsWith("moveTCPAbs"))
    {
      float x, y, z;
      if (sscanf(cmd.c_str(), "moveTCPAbs %f %f %f", &x, &y, &z) == 3)
        moveTCPAbs(x, y, z);
      else Serial.println("Usage: moveTCPAbs x y z");
    }

    else if (cmd.startsWith("moveTCPRel"))
    {
      float dx, dy, dz;
      if (sscanf(cmd.c_str(), "moveTCPRel %f %f %f", &dx, &dy, &dz) == 3)
        moveTCPRel(dx, dy, dz);
      else Serial.println("Usage: moveTCPRel dx dy dz");
    }

    else if (cmd.startsWith("pitch"))
    {
      float pitch_deg;
      if (sscanf(cmd.c_str(), "pitch %f", &pitch_deg) == 1)
      {
        double pitch_rad = pitch_deg * M_PI / 180.0;
        setPitch(pitch_rad);
      }
      else Serial.println("Usage: pitch val_deg");
    }

    else if (cmd == "toolopen") setGripper(true);
    else if (cmd == "toolclose") setGripper(false);
    else if (cmd == "moveHome") moveHome();
    else if (cmd == "keephori") keepHorizontal(); 

    else Serial.println("[ERR] Unknown command. Type 'help'.");
  }
}
