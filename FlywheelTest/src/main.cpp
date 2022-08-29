
#include "vex.h"

using namespace vex;

// DEVICES //////////////////////////////////
controller controllerPrim(primary);

// Motor 
motor LMBASE(PORT3, gearSetting::ratio6_1, false);


void flyWheel(double power)
{
  LMBASE.spin(fwd, power, pct);
}

int main()
 {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  LMBASE.setStopping(coast);
  while (1)
  {
    controllerPrim.Screen.setCursor(1,1);
    controllerPrim.Screen.clearLine();
    controllerPrim.Screen.print("RPM: %.0f", LMBASE.velocity(rpm));

    if (controllerPrim.ButtonL1.pressing())
      {
        LMBASE.spin(fwd, 100, pct);
      }
    else{
      LMBASE.stop();
    }

  }

}
