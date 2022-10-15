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
#include "functions.h"
#include "odom.h"

using namespace vex;

// A global instance of competition
competition Competition;


void pneumaticOn(){
  PWT.set(1);
}

void pneumaticOff(){
  PWT.set(0);
}

void toggleopneumatic(){
  if (PWT.value() == 1)
  {
    PWT.set(0);
  }
  else 
  {
    PWT.set(1);
  }
}




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


  INERTIAL.calibrate();
  // Wait until the inertial sensor finishes calibrating
  while (INERTIAL.isCalibrating())
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(210, 50, "Rot: CALIBRATING");
  }
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(210, 50, "Done calibrating");


  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


// AUTON ///////////////////////////////////////////////////////////////////

// full wp
void leftFull(void)
{
  
}

void rightFull(void)
{
  
}


// half wp
void leftHalf(void)
{
  
}

void rightHalf(void)
{
  
}

void skills()
{

}




// DRIVER ///////////////////////////////////////////////////////////////////
void usercontrol(void)
{

  // set up controller callbacks
  controllerPrim.ButtonA.pressed(toggleopneumatic);
  
  // User control code here, inside the loop
  while (1 == 1) {

    L1BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    L3BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    L4BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    //L4BASE.spin(fwd, controllerPrim.Axis3.value(), pct);


    R1BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    R3BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    R4BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    //R4BASE.spin(fwd, controllerPrim.Axis2.value(), pct);



    if (controllerPrim.ButtonR1.pressing())
    {
      R2BASE.spin(fwd, 100, pct);
    }
    else if (controllerPrim.ButtonR2.pressing())
    {
      R2BASE.spin(reverse, 100, pct);
    }
    else
    {
      R2BASE.stop(coast);
    }
    
    if (controllerPrim.ButtonR1.pressing())
    {
      L2BASE.spin(fwd, 100, pct);
    }
    else if (controllerPrim.ButtonR2.pressing())
    {
      L2BASE.spin(reverse, 100, pct);
    }
    else
    {
      L2BASE.stop(coast);
    }



    // There's multiple control schemes

    /*leftDrive(controllerPrim.Axis3.value());
    rightDrive(controllerPrim.Axis2.value());
    */

    

/*
    if (controllerPrim.ButtonR1.pressing())
    {
      R1BASE_ROLLER.spin(fwd, 100, pct);
    }
    else if (controllerPrim.ButtonR2.pressing())
    {
      R1BASE_ROLLER.spin(reverse, 100, pct);
    }
    else
    {
      R1BASE_ROLLER.stop(coast);
    }

    
    if (controllerPrim.ButtonL1.pressing())
    {
      L1BASE.spin(fwd, 100, pct);
      L2BASE.spin(fwd, 100, pct);
      R2BASE.spin(fwd, 100, pct);
    }
    else if (controllerPrim.ButtonL2.pressing())
    {
      L1BASE.spin(reverse, 100, pct);
      L2BASE.spin(reverse, 100, pct);
      R2BASE.spin(reverse, 100, pct);
    }
    else
    {
      L1BASE.stop(coast);
      L2BASE.stop(coast);
      R2BASE.stop(coast);
    }*/

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}





void values()
{

  Brain.Screen.setFillColor(color(10, 80, 30)); // Set background to green in rgb
  Brain.Screen.setPenColor(white); // Set text color to white

  // Display debug values such as position, rotation, encoder values, total distancel, etc.
  Brain.Screen.printAt(210, 30, "Pos: (%.1f, %.1f)     ", globalX, globalY);
  Brain.Screen.printAt(210, 50, "Rot: %.1f deg      ", getRotationDeg());
  Brain.Screen.printAt(210, 70, "Enc: L:%.1f R:%.1f    ", getLeftReading(), getRightReading());
  Brain.Screen.printAt(210, 90, "Dis: %.7f", getTotalDistance());

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


  // Prevent main from exiting with an infinite loop.
  while (true) {
    updatePosition(); // Update the odometry position
    // Show the debug values and the odometry display
    values();
    odomDisplay();
    task::sleep(10); // Wait some time between odometry cycles. Test making it shorter for better position estimates
  }
}
