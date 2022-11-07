/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  hello1 = 1;
  
  // fill screen with green bg
  Brain.Screen.setFillColor(color(10, 80, 30));
  Brain.Screen.setPenWidth(0);
  Brain.Screen.drawRectangle(0, 0, 480, 272);




  // Draw Y

  Brain.Screen.setPenColor(yellow);
  Brain.Screen.setPenWidth(40);
  Brain.Screen.drawLine(100, 125, 100, 220);
  Brain.Screen.drawLine(100, 130, 35, 35);
  Brain.Screen.drawLine(100, 130, 165, 35);

  INERTIAL.calibrate();
  // Wait until the inertial sensor finishes calibrating
  while (INERTIAL.isCalibrating())
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(210, 50, "Rot: CALIBRATING");
  }
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(210, 50, "Done calibrating");

  R3BASE.resetPosition();
  L3BASE.resetPosition();
  
  resetOdomPosition();
  resetTotalDistance();
  
  R3BASE.resetPosition();
  L3BASE.resetPosition();
  

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


// AUTON ///////////////////////////////////////////////////////////////////

// full wp
void leftFullOdom()
{
  resetOdomPosition();
  /*
  //forwardPID(-5, 30, 1000);
  //forwardPID(20, 60);
  setTarget(50, -40);
  moveToTarget(65, 15);
  turnPID(45, 35);
  // shoot
  wait(1000, msec);
  turnPID(-45, 35);

  setTarget(80, -80); // Right now
  passTarget(65, 15);
  setTarget(90, -90); // Right now
  passTarget(100, 15);
  setBase(15);*/





  setTargetPolar(32, -50); // 5 in towards -45 deg
  passTarget(40*1, 22*1);

  setTargetPolar(10, -50); // 5 in towards -45 deg
  passTarget(40*0.6, 22*0.6);

  turnPID(30.5,30);
  forwardPID(10,50);

  forwardPID(-10,50);
  turnPID(-45,30);


  stopBase();
  /*setTargetPolar(10, 0); // 10 in towards 0 deg
  moveToTarget(30*1, 25*1);

  turnPID(31, 30);
  wait(1000, msec);*/
  
  //turnPID(-40, 30);
  //forwardPID(75, 60);

}

void leftFull()
{
  //forwardPID(-100,30, 3000);
  //turnPID(-45,30);

  forwardInchesTimed(20,20,2000);
}


void rightFull()
{
  
}


// half wp
void leftHalf()
{

}

void rightHalf()
{
  
}

void setonSkills()
{


}


// DRIVER ///////////////////////////////////////////////////////////////////
void usercontrol(void)
{

  
  // set up controller callbacks befoe the code starts
  controllerPrim.ButtonDown.pressed(togglePto);
  controllerPrim.ButtonR1.pressed(catapultReset); // fire in a separate task

  
  // User control code here, inside the loop
  while (1 == 1) {

    L1BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    //L2BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    L3BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    L4BASE.spin(fwd, controllerPrim.Axis3.value(), pct);

    R1BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    //R2BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    R3BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    R4BASE.spin(fwd, controllerPrim.Axis2.value(), pct);

    if (PTO.value() == 1) // if power takeoff is out
    {
      // catapult
      if (controllerPrim.ButtonR2.pressing()) {
        resettingCata = false; // stop resetting cata if manually pressed
        R2BASE.spin(reverse, 100, pct);
      }
      else if (!resettingCata) // if not currently rresetting, stop when not prressing manual cata
      {
        R2BASE.stop(coast);
      }
      /* else if (controllerPrim.ButtonR1.pressing()) {
        R2BASE.spin(reverse, 100, pct);
      } else {
        R2BASE.stop(coast);
      }*/

      // Rollers/intake
      if (controllerPrim.ButtonL1.pressing()) {
        L2BASE.spin(fwd, 100, pct);
      } else if (controllerPrim.ButtonL2.pressing()) {
        L2BASE.spin(reverse, 100, pct);
      } else {
        L2BASE.stop(coast);
      }
    }
    else { // power takeoff in - all 8 motors. include 2
      L2BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
      R2BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    }

    /*
    // Tap the screen to move to a corresponding point on the field
    if (Brain.Screen.pressing())
    {
      setTarget(getScreenTouchX(), getScreenTouchY());
      //turnToTarget(30);
      moveToTarget(35, 20);
    }*/
  
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}



void values()
{

  //Brain.Screen.setFillColor(color(10, 80, 30)); // Set background to green in rgb
  Brain.Screen.setPenColor(white); // Set text color to white

  // Display debug values such as position, rotation, encoder values, total distancel, etc.
  Brain.Screen.printAt(210, 30, "Pos: (%.1f, %.1f)     ", getGlobalX(), getGlobalY());
  Brain.Screen.printAt(210, 50, "Rot: %.1f deg      ", getRotationDeg());
  Brain.Screen.printAt(210, 70, "Enc: L:%.1f R:%.1f    ", getLeftReading(), getRightReading());
  Brain.Screen.printAt(210, 90, "Dis: %.7f", getTotalDistance());
  //Brain.Screen.printAt(210,110, "Rot: %.2f deg      ", encoderL.rotation(deg));


  //Brain.Screen.printAt(210, 130, "isStopped: %d   ", isStopped());

  /* Brain.Screen.printAt(210, 110, "lir: %.1f  aaom: %.1f", lastInertialRadians * toDegrees, absoluteAngleOfMovement * toDegrees);
  Brain.Screen.printAt(210, 130, "dth: %.1f  dd:%.1f", deltaTheta * toDegrees, deltaDistance);
  Brain.Screen.printAt(210, 150, "dl:%.1f  dr:%.1f", deltaLeft, deltaRight);
  Brain.Screen.printAt(210, 170, "dx:%.1f  dy:%.1f", deltaX, deltaY);*/
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(leftFull);
  Competition.drivercontrol(usercontrol);
  newFunction();
  // Run the pre-autonomous function.
  pre_auton();


  //controllerPrim.ButtonDown.pressed(toggleopneumatic);

  // Prevent main from exiting with an infinite loop.
  // A lot of asyncronous tasks separate from the auton and driver task occur here

  //task c(catapultPID); // control the cata's PID separate from autonomous task


  while (true) {

    //catapultPID(); // control the cata's PID separate from autonomous task

    // Run these independently of auton and driver tasks
    updatePosition(); // Update the odometry position
    // Show the debug values and the odometry display
    odomDisplay();

    if (Brain.Screen.pressing())
    {
      INERTIAL.calibrate();
    }

    values();
    task::sleep(10); // Wait some time between odometry cycles. Test making it shorter for better position estimates
  }
}