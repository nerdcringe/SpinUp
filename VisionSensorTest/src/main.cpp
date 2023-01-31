/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\hererjd23                                        */
/*    Created:      Tue Jan 31 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision               vision        14              
// RBASE                motor_group   18, 19          
// LBASE                motor_group   13, 17          
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;


double goalCorrectR = 0;
double goalCorrectL = 0;


void turnToGoal()
{
  int x = 0;
  int center = 158;// The x coordinate for the center of the vision sensor
  int OKError = 25; //Used to set a range of values to count is being just in front
  int error;

  double Kp = 0.3;

  while (true) {
    Vision.takeSnapshot(Vision__REDGOAL);
    if (Vision.largestObject.exists) {
      x = Vision.largestObject.centerX;
      error = center - x;
      if (fabs(error) > OKError){
        goalCorrectR = error * Kp + 1;
        goalCorrectL = -error * Kp - 1;
      } else {
        Controller1.rumble(".");
        break;
      }
      Controller1.Screen.clearScreen();
      task::sleep(50);
      Controller1.Screen.print("GOAL!!!   ");

    } else {

      Vision.takeSnapshot(Vision__BLUEGOAL);
      if (Vision.largestObject.exists) {
        x = Vision.largestObject.centerX;
        error = center - x;
        if (fabs(error) > OKError) {
          goalCorrectR = error * Kp + 1;
          goalCorrectL = -error * Kp - 1;
        } else {
          Controller1.rumble(".");
          break;
        }
        Controller1.Screen.clearScreen();
        task::sleep(50);
        Controller1.Screen.print("GOAL!!!   ");
      } else {
        break;
      }
    }
  }
  goalCorrectR = 0;
  goalCorrectL = 0;
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Controller1.ButtonR1.pressed(turnToGoal);
  
  while(true){
    
    RBASE.spin(fwd, Controller1.Axis2.value() + goalCorrectR, pct);
    LBASE.spin(fwd, Controller1.Axis3.value() + goalCorrectL, pct);
    task::sleep(20);
  }
}
