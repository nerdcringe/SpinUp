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

// define your global instances of motors and other devices here

// DEVICES ///////////////////////////////////


  // motors //

  motor L1BASE(PORT15, true); // front
  motor L2BASE(PORT17, true); // back top
  motor L3BASE(PORT14); // back mid
  motor L4BASE(PORT11, true); // back bottom

  motor R1BASE(PORT18); // front
  motor R2BASE(PORT13); 
  motor R3BASE(PORT19, true);
  motor R4BASE(PORT9);


  // Sensors & more //

  inertial INERTIAL(PORT14);

  triport Triport(PORT22); // Get reference for three-wire ports on brain

  digital_out PWT(Triport.A);


  triport TriportExt(PORT9); // Get reference for three wire extender
  encoder encoderL(TriportExt.A); // left tracking wheel
  encoder encoderR(TriportExt.C); // right tracking wheel
  encoder encoderS(TriportExt.E); // sideways tracking wheel. Testing to see if we need this to deal with drift

  controller controllerPrim(controllerType::primary);


// MATH FUNCTIONS /////////////////////////////////////////////


  // OLD FUNCTIONS FOR MOTOR ENCODERS
  /*
  // Convert distance to move to ticks to rotate base motors
  double inchesToTicks(double inches)
  {
    return inches * (360 / WHEEL_CIRCUMFERENCE);
  }

  double ticksToInches(double ticks)
  {
    return ticks * (WHEEL_CIRCUMFERENCE / 360);
  }*/



// STANDARD MOVEMENT ////////////////////////////

  // Set the speeds of different sides of the base
  void leftDrive(double power)
  {
    // check which motor setup is being used based on pto
    L1BASE.spin(fwd, power, pct);
    L2BASE.spin(fwd, power, pct);
    L3BASE.spin(fwd, power, pct);
    L4BASE.spin(fwd, power, pct);
  }

  void rightDrive(double power)
  {
    R1BASE.spin(fwd, power, pct);
    R2BASE.spin(fwd, power, pct);
    R3BASE.spin(fwd, power, pct);
    R4BASE.spin(fwd, power, pct);
  }

  void drive(double power)
  {
    leftDrive(power);
    rightDrive(power);
  }

  void stopBase()
  {
    L1BASE.stop(coast);
    L2BASE.stop(coast);
    L3BASE.stop(coast);
    L4BASE.stop(coast);

    R1BASE.stop(coast);
    R2BASE.stop(coast);
    R3BASE.stop(coast);
    R4BASE.stop(coast);
  }

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

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // There's multiple control schemes

    leftDrive(controllerPrim.Axis3.value());
    rightDrive(controllerPrim.Axis2.value());


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


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  controllerPrim.ButtonA.pressed(toggleopneumatic);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
