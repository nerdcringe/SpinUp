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



// MISC METHODS /////////////////////////////////////////////////////////////////////////

void toggleLock() { LOCK.set(!LOCK.value()); } // invert the value
void lockOn() { LOCK.set(1); }
void lockOff() { LOCK.set(0); }
void manspread() { ENDGAME.set(!ENDGAME.value()); }// deploy endgame mech

void setIntake(double speed){ INTAKE.spin(fwd,speed,velocityUnits::pct); }
void stopIntake() { INTAKE.stop(coast); }


bool resettingSling = false; // set to false to cancel reset

void resetSling() {
  resettingSling = true;
  XBOW.spin(fwd, 100, velocityUnits::pct);
  // spin until limit is pressing or reset is canceled
  waitUntil(CATALIMIT.pressing() || !resettingSling);
  task::sleep(200);
  lockOn();
  task::sleep(150);
  XBOW.stop(brakeType::coast);
  
  XBOW.rotateTo(0, deg, 100, velocityUnits::pct);
  resettingSling = false;
}

int resetSlingAsync() {
  resetSling();
  return 0;
}

void shootSling(){
  lockOff();
  wait(500, msec);
  resetSling();
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
  Brain.Screen.setFillColor(color(10, 80, 30));
  Brain.Screen.drawRectangle(0, 0, 480, 272);

  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(210, 50, "Done calibrating");

  // Draw Y
  Brain.Screen.setPenColor(yellow);
  Brain.Screen.setPenWidth(40);
  Brain.Screen.drawLine(100, 125, 100, 220);
  Brain.Screen.drawLine(100, 130, 35, 35);
  Brain.Screen.drawLine(100, 130, 165, 35);

  Brain.Screen.setPenColor(white);
  resetTotalDistance();
}



// AUTON ///////////////////////////////////////////////////////////////////

void rollerAuton(){
  // have to reset these manually before every auton
  resetTotalDistance();
  resettingSling = false; // stop resetting cata if manually pressed


  forwardInchesTimed(-3.5, 15, 750);
  forwardInchesTimed(3, 25, 3000);

  
}

void rightHalf() {
  // have to reset these manually before every auton
  resetTotalDistance();
  resettingSling = false; // stop resetting cata if manually pressed

  
  forwardInchesTimed(-3.5, 15, 750);
  forwardInchesTimed(3, 25, 3000);
  task b(resetSlingAsync);
  turnPID(46, 25);
  forwardPID(32.5, 50);
  turnPID(-45, 25);
  shootSling();

}

void pidTest(){
  //forwardPID(30, 40); // aight
  //wait(3, sec);
  //turnPID(45, 20);
  turnPID(90, 25);

}



// DRIVER ///////////////////////////////////////////////////////////////////
void usercontrol(void) {
  // set up controller callbacks befoe the code starts
  // these un the function in a new thread so it doesn't interrupt the rest of driver code
  controllerPrim.ButtonB.pressed(toggleLock);
  //controllerPrim.ButtonR1.pressed(catapultResetDriver); // fire in a separate task
  controllerPrim.ButtonX.pressed(manspread); // deploy endgame mech
  controllerPrim.ButtonLeft.pressed(resetSling);
  //controllerPrim.ButtonDown.pressed(shootSling);

  L1BASE.setStopping(brakeType::coast);
  L2BASE.setStopping(brakeType::coast);
  L3BASE.setStopping(brakeType::coast);
  //L4BASE.setStopping(brakeType::coast);

  R1BASE.setStopping(brakeType::coast);
  R2BASE.setStopping(brakeType::coast);
  R3BASE.setStopping(brakeType::coast);
  //R4BASE.setStopping(brakeType::coast);

  // User control code here, inside the loop
  while (1 == 1) {
    L1BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    L2BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    L3BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    //L4BASE.spin(fwd, controllerPrim.Axis3.value(), pct);

    R1BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    R2BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    R3BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    //R4BASE.spin(fwd, controllerPrim.Axis2.value(), pct);

    // crossbow reload buttons
    if (controllerPrim.ButtonL1.pressing()) {
      resettingSling = false;
      XBOW.spin(forward, 100, pct); // backward just in case
    } else if (controllerPrim.ButtonL2.pressing()) {
      resettingSling = false;
      XBOW.spin(reverse, 100, pct); // forward, only one we really need
    } else if (!resettingSling) {
      XBOW.stop(); // stop only if not doing auto reset in task started by button.pressed()
    }

    // intake controls
    if (controllerPrim.ButtonR1.pressing()) {
      INTAKE.spin(forward, 100, pct); // backward just in case
    } else if (controllerPrim.ButtonR2.pressing()) {
      INTAKE.spin(reverse, 100, pct); // forward, only one we really need
    } else {
      INTAKE.stop();
    }
  
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}



void values() {
  Brain.Screen.setPenColor(white); // Set text color to white

  Brain.Screen.printAt(0, 20, "name: %d   ", getRotationDeg());

  Brain.Screen.printAt(210, 30, "Rot: %.2f deg       ", getRotationDeg());
  Brain.Screen.printAt(210, 50, "Dist: %.2f         ", getTotalDistance());
  Brain.Screen.printAt(210, 70, "%d  %d   ", (int)L1BASE.velocity(pct), (int)R1BASE.velocity(pct));
  Brain.Screen.printAt(210, 90, "%d  %d   ", (int)L2BASE.velocity(pct), (int)R2BASE.velocity(pct));
  Brain.Screen.printAt(210, 110,"%d  %d   ", (int)L3BASE.velocity(pct), (int)R3BASE.velocity(pct));
  Brain.Screen.printAt(210, 130,"sling dist %d    ", (int)XBOW.rotation(deg));
  Brain.Screen.printAt(300, 90,"LOCK %d    ", (int)LOCK.value());
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(rightHalf);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  // A lot of asyncronous tasks separate from the auton and driver task occur here
  while (true) {
    // Run these independently of auton and driver tasks
    if (Brain.Screen.pressing()) {
      INERTIAL.calibrate();
    }

    // Show the debug values and the odometry display
    values();
    task::sleep(20);
  }
}