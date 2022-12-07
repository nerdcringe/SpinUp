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
  XBOW.stop(brakeType::coast);
  resettingSling = false;
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


  setBase(5); // move into rollers
  task::sleep(500);//450
  holdBase();

  //roller(275,100);
 
 // back up from roller and turn towards center of field
  forwardPID(-1.5, 35);
}

void leftFull(){
  // PID test
  //forwardPID(24, 35);

  
  //INERTIAL.setRotation(0, deg);
  
  //turnPID(89.875, 30);
  //turnPID(-45, 28);

  //forwardInchesTimed(1.5, 5, 750);
  //pto6();


  // have to reset these manually before every auton
  resetTotalDistance();
  resettingSling = false; // stop resetting cata if manually pressed


  setBase(5); // move into rollers
  task::sleep(300);//450
  holdBase();

  //roller(275,100);
  //setIntake(50);

          //wait(200, msec);
          //setIntake(0);
         //stopBase();
          

 // back up from roller and turn towards center of field
  forwardPID(-2.8, 35);
  turnPID(-45.6, 25);

  setIntake(100);
 //forwardPIDIncr(-30, 70); // go to center
  setIntake(0);
  task::sleep(50);
  
  //gyroTurn(-45, 20);
  //setIntake(100);
  //forwardPID(-30, 55); // go to center
  turnPID(44.15, 22, 1750); // turn to shoot
  forwardInches(-0.9, 16);
  setIntake(100);
  task::sleep(500);
  setIntake(0);
  task::sleep(100);
  
  R2BASE.spin(reverse, 100, pct); // shoot
  wait(600, msec);
  //task pog(catapultReset);

  setIntake(100);
  task::sleep(750);
  setIntake(0);
  //task::sleep(100);
  /* // shoot twice
  R2BASE.spin(reverse, 100, pct); // shoot
  wait(800, msec);
  task pog(catapultReset);
  wait(250, msec);*/

  forwardInches(0.75, 22);

  //turnPID(-44, 28);
  gyroTurn(-46.5, 25);
  forwardInches(-28, 100);
  
  gyroTurn(91, 30);
  //turnPID(90, 30);

  forwardInchesTimed(4.25, 50, 2500); // move into rollers
  setBase(5); // move into rollers
  task::sleep(500);
  holdBase();

  //roller(350, 100);
  stopBase();
  
}

void rightHalfGood() {
  resetTotalDistance();  
  //pto6();


  //forwardInchesTimed(1.5, 5, 750);
  setBase(5); // move into rollers
  task::sleep(300);
  holdBase();

  //roller(275,100);
  //setIntake(50);

          //wait(200, msec);
          //setIntake(0);
         //stopBase();
        
 // back up from roller and turn towards center of field
  //forwardPID(-2.8, 35);
  forwardPID(-2.8, 20);
  turnPID(45.6, 25);

  setIntake(100);
  //forwardPIDIncr(-30, 70); // go to center
  setIntake(0);
  task::sleep(50);
  
  //gyroTurn(-45, 20);
  //setIntake(100);
  //forwardPID(-30, 55); // go to center
  //turnPID(-44.15, 22, 1750); // turn to shoot
  turnPID(-43.75, 22, 1750); // turn to shoot
  forwardInches(-0.8, 13);
  setIntake(100);
  task::sleep(500);
  setIntake(0);
  task::sleep(100);
  
  R2BASE.spin(reverse, 100, pct); // shoot
  wait(600, msec);
  //task pog(catapultReset);

  task::sleep(1000);
  // pick up to shoot again
  setIntake(100);
  forwardInches(6, 17);
  forwardInches(-6, 17);
  
  gyroTurn(-75, 22); // turn to other disk
  
  setIntake(100);
  forwardInches(8, 17);
  forwardInches(-7, 17);
  
  //turnPID(-44, 22, 2500); // turn to shoot
  turnPID(-44, 22, 2500); // turn to shoot
  setIntake(0);
  forwardInches(-0.7, 12);

  R2BASE.spin(reverse, 100, pct); // shoot
  wait(600, msec);
  //task m(catapultReset);

  stopBase();


}

void setonSkills(){
  resetTotalDistance();  
  //pto6();

  //forwardInchesTimed(1.5, 5, 750);
  setBase(5); // move into rollers
  task::sleep(300);
  holdBase();

  //roller(450,100);
  //setIntake(50);

          //wait(200, msec);
          //setIntake(0);
         //stopBase();
        
 // back up from roller and turn towards center of field
  //forwardPID(-2.8, 35);
  forwardInches(-1.5, 20);
  turnPID(138, 25);
  setIntake(100);
  forwardInches(6, 20);
  
  gyroTurn(145, 20);
  forwardInches(6, 20);
  turnPID(90, 26);
  //setIntake(0);

 // move into rollers
 
  setIntake(0);
  forwardInchesTimed(2.25, 15, 1500);
  setBase(5);
  task::sleep(275);
  holdBase();

  //roller(475, 100);

  forwardInches(-1.5, 15);
  turnPID(-2.3, 27);
  setIntake(100);
  forwardPID(-26, 45);
  
  gyroTurn(0.5, 23);
  R2BASE.spin(reverse, 100, pct); // shootol
  wait(600, msec);
  //task m(catapultReset);
  
  wait(100, msec);

  forwardInches(4, 20);
  
  turnPID(-60, 23);
  forwardPID(17, 30);
  setIntake(100);
  turnPID(-147, 27);

}

void pidTest(){
  //forwardPID(30, 40); // aight

  turnPID(-90, 40);

}



// DRIVER ///////////////////////////////////////////////////////////////////
void usercontrol(void) {
  // set up controller callbacks befoe the code starts
  // these un the function in a new thread so it doesn't interrupt the rest of driver code
  controllerPrim.ButtonB.pressed(toggleLock);
  //controllerPrim.ButtonR1.pressed(catapultResetDriver); // fire in a separate task
  controllerPrim.ButtonX.pressed(manspread); // deploy endgame mech
  //controllerPrim.ButtonLeft.pressed(resetSling);
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

  Brain.Screen.printAt(210, 30, "Rot: %.2f deg       ", getRotationDeg());
  Brain.Screen.printAt(210, 50, "Dist: %.2f         ", getTotalDistance());
  Brain.Screen.printAt(210, 70, "%d  %d   ", (int)L1BASE.velocity(pct), (int)R1BASE.velocity(pct));
  Brain.Screen.printAt(210, 90, "%d  %d   ", (int)L2BASE.velocity(pct), (int)R2BASE.velocity(pct));
  Brain.Screen.printAt(210, 110,"%d  %d   ", (int)L3BASE.velocity(pct), (int)R3BASE.velocity(pct));
  Brain.Screen.printAt(210, 130,"sling dist %d    ", (int)XBOW.rotation(deg));
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(pidTest);
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