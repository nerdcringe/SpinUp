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

  printf("Callibrating \n"); // print to computer terminal
  INERTIAL.calibrate();
  // Wait until the inertial sensor finishes calibrating
  while (INERTIAL.isCalibrating()) {
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(fontType::mono40);
    Brain.Screen.printAt(100, 75, "INERTIAL SENSOR");
    Brain.Screen.printAt(100, 125, "CALIBRATING...");
    Brain.Screen.setFont(fontType::mono20);
  }

  // fill screen background
  Brain.Screen.setPenWidth(0);
  Brain.Screen.setFillColor(color(20, 110, 50));
  Brain.Screen.drawRectangle(0, 0, 480, 272);

  // Draw Y
  Brain.Screen.setPenColor(yellow);
  Brain.Screen.setPenWidth(40);
  int x = 385; // amount to shift to the side
  Brain.Screen.drawLine(x, 125, x, 220);
  Brain.Screen.drawLine(x, 130, x-65, 35);
  Brain.Screen.drawLine(x, 130, x+65, 35);

  Brain.Screen.setPenColor(white);
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

    
      double leftSpeed = controller1.Axis3.value();
      double rightSpeed = controller1.Axis2.value();
      LBASE.spin(forward, leftSpeed, velocityUnits::pct);
      RBASE.spin(forward, rightSpeed, velocityUnits::pct);


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


 // useless counter to demonstrate button.pressed() callbacks
int counter = 0;
void countUp() {
  counter += 1;
}


//
// Main will set up the competition functions and callbacks.
//
int main() {

  controller1.ButtonB.pressed(countUp);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);

    controller1.Screen.setCursor(1, 1);
    controller1.Screen.print("%.2f  ", INERTIAL.rotation(rotationUnits::deg));

    // Show the debug values
    // %d is for ints (whole numbers), %f is for floats/doubles (decimals)
    Brain.Screen.setFont(fontType::mono30);
    Brain.Screen.printAt(10, 30, "Rot: %.2f  ", INERTIAL.rotation(deg));
    Brain.Screen.printAt(10, 60, "Dist: %.2f  ", LBASE.rotation(deg));
    Brain.Screen.setFont(fontType::mono20);
    Brain.Screen.printAt(10, 90,"DEBUG %d  ", true);
    Brain.Screen.printAt(10, 110,"VALUES %d  ", 35);
    Brain.Screen.printAt(10, 130,"GO %f  ", 1.5);
    Brain.Screen.printAt(10, 150,"HERE %.2f  ", 3.14159); // round to 2 decimal places
    Brain.Screen.printAt(10, 170,"BUTTON A PRESSING %d  ", controller1.ButtonA.pressing());
    Brain.Screen.printAt(10, 190,"COUNTER %d  ", counter);
  }
}
