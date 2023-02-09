/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\hererjd23                                        */
/*    Created:      Thu Feb 02 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "testVision.h"

using namespace vex;

double goalCorrectR = 0;
double goalCorrectL = 0;



void goalCorrect(){

int x = 0;
int center = 158;// The x coordinate for the center of the vision sensor
int OKError = 10; //25; //Used to set a range of values to count is being just in front.
int curError = 0;
double kp = 0.15;
double maxSpeed = 30;
double minSpeed = 3; 

while(true){
  
  Controller1.Screen.setCursor(0,0);

  VisionSensor.takeSnapshot(BLUEGOAL);
  if(!VisionSensor.largestObject.exists ){
    VisionSensor.takeSnapshot(REDGOAL);
  }

  if (VisionSensor.largestObject.exists) {
    x = VisionSensor.largestObject.centerX;
    curError = center - x;
    if( x < (center- OKError)) //If the object is to the left of center
    {
      goalCorrectR = abs(curError) * kp;
      if(goalCorrectR > maxSpeed){goalCorrectR = maxSpeed;}
      if(goalCorrectR < minSpeed){goalCorrectR = minSpeed;}
      goalCorrectL = -abs(curError) * kp;
      if(goalCorrectL < -maxSpeed){goalCorrectL = -maxSpeed;}
      if(goalCorrectL > -minSpeed){goalCorrectL = -minSpeed;}
      Controller1.Screen.print("LEFT!!!   ");
    } 
    else if (x> center + OKError) //If the object is to the right of center
    {
      goalCorrectR = -abs(curError) * kp;
      if(goalCorrectR < -maxSpeed){goalCorrectR = -maxSpeed;}
      if(goalCorrectR > -minSpeed){goalCorrectR = -minSpeed;}
      goalCorrectL = abs(curError) * kp;
       if(goalCorrectL > maxSpeed){goalCorrectL = maxSpeed;}
       if(goalCorrectL < minSpeed){goalCorrectL = minSpeed;}
      Controller1.Screen.print("RIGHT!!!   ");
    } 
    else //The object is not to the right of center and not to the left of center
    {
      goalCorrectR = 0;
      goalCorrectL = 0;
      Controller1.Screen.print("GOAL!!!   ");
      break;
    }

  }
  else
  {
    Controller1.Screen.print("Hello");
    goalCorrectR = 0;
    goalCorrectL = 0;
    break;
  }
}
}


void goalPID(){ 
   // PID //
  double kp = 0.17;
  double kd = 1.0;

  int center = 158;// The x coordinate for the center of the vision sensor
  int OKError = 10; //25; //Used to set a range of values to count is being just in front.
  int OKTime = 100; // time needed to settle in okerror to exit
  int maxSpeed = 30;

  int x = 0;
  int curError = 0;
  int lastError = 0;
  int derivative = 0;
  int curTime = 0;
  

  while(true){
    Controller1.Screen.setCursor(0,0);
    VisionSensor.takeSnapshot(BLUEGOAL);
    if(!VisionSensor.largestObject.exists ){
      VisionSensor.takeSnapshot(REDGOAL);
    }

    if (VisionSensor.largestObject.exists) {

      x = VisionSensor.largestObject.centerX;
      curError = center - x;
      derivative = curError - lastError;

      if(abs(curError) > OKError){
        double speed = curError * kp + derivative * kd;
        if(speed > maxSpeed){speed = maxSpeed;}
        if(speed < -maxSpeed){speed = -maxSpeed;}
        
        goalCorrectR = speed;
        goalCorrectL = -speed;

        curTime = 0;
        
      } else {
        curTime = curTime + 10;
      }

      if  (curTime > OKTime) { // stay within okerror for long enough
        Controller1.rumble(".");
        goalCorrectR = 0;
        goalCorrectL = 0;
        break;
      }

      lastError = curError;
    } else {
      Controller1.Screen.print("Hello");
      goalCorrectR = 0;
      goalCorrectL = 0;
      break;
    }

    wait(10, msec);
  }
  
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

Controller1.ButtonR1.pressed(goalPID);


while (true)
{
   RBASE.spin(directionType::fwd, goalCorrectR + Controller1.Axis2.value(), velocityUnits::pct);
   LBASE.spin(directionType::fwd, goalCorrectL + Controller1.Axis3.value(), velocityUnits::pct);
   task::sleep(100);
}

}


  

