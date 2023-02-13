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
void toggleLock() {
  LOCK.set(!LOCK.value()); // invert the value
}
void lockOn() { LOCK.set(0); }
void lockOff() { LOCK.set(1); }


void togglePto() {
  PTO.set(!PTO.value()); // invert the value
}
void ptoOn() { PTO.set(1); }
void ptoOff() { PTO.set(0); }


void toggleMunch() {
  MUNCHER.set(!MUNCHER.value()); // invert the value
}
void munchOn() { MUNCHER.set(1); }
void munchOff() { MUNCHER.set(0); }


void manspread() {
  ENDGAME.set(!ENDGAME.value());// deploy endgame mech
}


void setIntake(double speed){ INTAKE.spin(reverse,speed,velocityUnits::pct); }
void stopIntake() { INTAKE.stop(coast); }


void roller(double dist, double speed) { INTAKE.spinFor(reverse,dist, rotationUnits::deg, speed,velocityUnits::pct); }


bool resettingSling = false; // set to false to cancel reset

void resetSling() {
  resettingSling = true; // reset can be canceled with manual buttons in driver control

  //lockOff();
  // pull back sling
  XBOW.spin(fwd, 100, velocityUnits::pct);
  INTAKE.spin(fwd, 100, velocityUnits::pct); // intake helps pull back to reduce stalling
  lockOff();

  // spin until limit switch is pressing, or reset is canceled
  waitUntil(CATALIMIT.pressing() || !resettingSling);
  task::sleep(300);
  lockOn(); // lock the disc holder in place with pneumatic
  task::sleep(150);
  XBOW.stop(brakeType::coast);
  INTAKE.stop(brakeType::coast);

  // unwind string to original position
  INTAKE.spin(reverse, 100, pct);
  XBOW.rotateTo(0, deg, 100, velocityUnits::pct); // xbow unwinds by itself without intake
  INTAKE.stop();
  resettingSling = false;
}


void launchResetSling() {
  resettingSling = true; // reset can be canceled with manual buttons in driver control
  lockOff();
  task::sleep(250);

  // pull back sling
  XBOW.spin(fwd, 100, velocityUnits::pct);
  INTAKE.spin(fwd, 100, velocityUnits::pct); // intake helps pull back to reduce stalling


  // spin until limit switch is pressing, or reset is canceled
  waitUntil(CATALIMIT.pressing() || !resettingSling);
  task::sleep(300);
  lockOn(); // lock the disc holder in place with pneumatic
  task::sleep(150);
  XBOW.stop(brakeType::coast);
  INTAKE.stop(brakeType::coast);

  // unwind string to original position
  INTAKE.spin(reverse, 100, pct);
  XBOW.rotateTo(0, deg, 100, velocityUnits::pct); // xbow unwinds by itself without intake
  INTAKE.stop();
  resettingSling = false;
}

//  call in task for auto
int resetSlingAsync(){
  resetSling();
  return 0;
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

  // Draw Y
  Brain.Screen.setPenColor(yellow);
  Brain.Screen.setPenWidth(40);
  int x = 285; // amount to shift to the side
  Brain.Screen.drawLine(100+x, 125, 100+x, 220);
  Brain.Screen.drawLine(100+x, 130, 35+x, 35);
  Brain.Screen.drawLine(100+x, 130, 165+x, 35);

  Brain.Screen.setPenColor(white);
  resetTotalDistance();
}


// AUTONS ///////////////////////////////////////////////////////////////////


// #LEFT WP SHOOT FROM ROLLER //
void leftFull() {
  // ROLLER #1 ////////////////
  
  setBase(-10);
  task::sleep(350);
  setBase(-2);

  roller(300, 100);
  forwardInches(13, 19);
  turnPID(8, 10);
  task::sleep(250);
  lockOff();
  task::sleep(300);
  turnPID(-45, 22);
  //gyroTurn(-43, 15);
  // //turnPID(-44, 23);
  
  task::sleep(500);
  forwardPIDGradual(123, 65); //125 in


  //turnL(240, 27);
  turnL(240, 22);
  //gyroTurn(135, 30);//turnPID(135, 24);

  setIntake(100);
  //revConst(150, 15, 3000);
  
  revConst(120, 15, 4000);
  //revConst(30, 6, 5000);
  stopIntake();

  controllerPrim.rumble(".");
  //forwardInches(1, 1)
  
  //revConst(200, 20, 1500);

  //setBase(-2);
  //roller(375, 100);
  //roller(425, 100);
  //stopBase();
}


// LEFT WP SHOOT FROM MIDFIELD
void leftFullMid() {
  // ROLLER #1 ////////////////
  setBase(-10);
  task::sleep(350);
  setBase(-2);

  //stopBase();

  roller(300, 100);
  //lockOff();
  //task s(resetSlingAsync);
  forwardInches(5, 22);
  //gyroTurnSlow(-38, 20);
  //gyroTurnSlow(-47, 25);
  turnPID(-50, 21);

  forwardPIDGradual(69, 55);
  // SHOOT AT HIGH GOAL //////////
  //gyroTurnSlow(43, 32);
  turnPID(42.5, 25);
  task::sleep(100);
  forwardInches(-2, 13);
  task::sleep(200);
  toggleLock();
  task::sleep(200);
  forwardInches(2, 13);

  // TURN TO ROLLER #2  /////////
  //turnPID(-35, 32);
  //turnR(155, 18);
  turnPID(-34.5, 25);
   forwardPIDGradual(68, 60); // 65 in
  revConst(50, 22);

   turnL(210, 27);//220
  revConst(200, 33, 550);
  stopBase();
  roller(340, 100);
}


// RIGHT HALF 2 DISCS //
void right2()
{
  // ROLLER /////////////////////
  forwardInches(-20, 37);
  turnPID(-90, 35);
  forwardInchesTimed(-7.5, 32, 99999);
  roller(220, 100);
  //forwardInches(10, 20);
  forwardInches(13, 25);
  task::sleep(100);
  // SHOT 1 ////////////////////////////////
  turnPID(-98, 28);
  task::sleep(150);
  lockOff();
  task::sleep(150);
}



        int delayIntake() {
          //task::sleep(1000);
          waitUntil(!resettingSling); // DONT TURN ON INTAKE UNTIL UNWIND IS COMPLETE
          task::sleep(50);
          setIntake(100);
          return 0;
        }

// RIGHT HALF 5 DISCS //
void right5() {
  // ROLLER /////////////////////
  forwardInches(-20, 37);
  turnPID(-90, 35);
  forwardInchesTimed(-7.5, 32, 3000);
  roller(220, 100);
  //forwardInches(10, 20);
  forwardInches(13, 26);
  task::sleep(100);
  // SHOT 1 ////////////////////////////////
  turnPID(-98, 28);
  task::sleep(150);
  lockOff();
  task::sleep(150);
  
  task banana(resetSlingAsync);
  turnPID(-110, 27);

  forwardInches(-11.5, 30);
  
  turnPID(-44.5, 23);
  task::sleep(150);
  
  task pineapple(delayIntake);
  forwardPIDGradual(67.5, 55);
  turnPID(-135, 25);
  stopIntake();
  task::sleep(150);
  //forwardInches(5, 12);
  lockOff();

}



        int intakeDelay() {
          task::sleep(1250);
          setIntake(100);
          return 0;
        }

        int stopIntakeDelay() {
          task::sleep(500);
          stopIntake();
          return 0;
        }

// LEFT ROLLER //
void leftRoller() {
  // LB ROLLER ////////////////
  setBase(-10);
  task::sleep(350);
  setBase(-2);
  roller(500, 100);
  //wait(0.15, sec);
  //task jsdhgkj(intakeDelay);
  forwardInches(3, 22);
  //task::sleep(500);
}

// LEFT HALF 2 DISCS
void left2() {
  // ROLLER #1 ////////////////
  setBase(-10);
  task::sleep(350);
  setBase(-2);

  //stopBase();
  INTAKE.setTimeout(5000, msec);
  roller(360, 100);
  //lockOff();
  //task s(resetSlingAsync);
  forwardInches(7, 19);
  turnPID(6, 24);
  task::sleep(500);
  lockOff();
}


// SKILLS //
void skills() {
  // LB ROLLER ////////////////
  setBase(-10);
  task::sleep(350);
  setBase(-2);
  roller(360, 100);
  //wait(0.15, sec);
  task jsdhgkj(intakeDelay);
  forwardInches(26, 22);
  task::sleep(500);

  // RR ROLLER ///////////////
  turnPID(-90, 37);
  task gsohgo(stopIntakeDelay);
  forwardInchesTimed(-25, 45, 1000);
  setBase(-2);
  roller(360, 100);
  stopBase();

  // B GOAL #1 ///////////////
  forwardInches(10, 30);
  turnPID(-3, 26);
  forwardPIDGradual(45, 45);
  lockOff();
  task::sleep(500);

  // INTAKE RED LINE OF 3 /////////////////
  task rbvhsvhdv(resetSlingAsync);
  forwardPID(-36, 30);
  task::sleep(3000);
  turnPID(-45,30);
  setIntake(100);
  forwardPIDGradual(64, 40);
  task::sleep(400);
  forwardInches(-3.5, 10);

  // BLUE GOAL #2 //////////// 
  turnPID(45, 26);
  stopIntake();
  forwardInches(9, 12);
  task::sleep(100);
  lockOff();
  task::sleep(100);
  forwardInches(-5, 12);
  turnPID(-55, 30);
  task::sleep(100);

  // LR ROLLER ///////////////
  forwardPIDGradual(56, 55);
  turnPID(-180, 25);
  forwardInchesTimed(-20, 25, 3500);
  roller(360, 100);
  forwardPID(25, 30);

  // RB ROLLER ///////////////
  turnPID(-270, 25);
  forwardInchesTimed(-32, 30, 6000);
  roller(360, 100);

  // ENDGAME ////////////////
  forwardInches(29, 26);
  turnPID(-270+46, 23);
  forwardInches(-30, 26);
  manspread(); // usually hits 20-24 tiles

  // tiles: 21
  // RockRidge - #1: 104
}

// SKILLS FOR STIFF ROLLERS //
void skillsStiff() {
  // LB ROLLER ////////////////
  setBase(-10);
  task::sleep(350);
  setBase(-2);
  roller(500, 100);
  //wait(0.15, sec);
  task jsdhgkj(intakeDelay);
  forwardInches(26, 22);
  task::sleep(500);

  // RR ROLLER ///////////////
  turnPID(-90, 37);
  task gsohgo(stopIntakeDelay);
  forwardInchesTimed(-25, 45, 1000);
  setBase(-2);
  roller(520, 100);
  stopBase();

  // B GOAL #1 ///////////////
  forwardInches(10, 30);
  turnPID(-3, 26);
  forwardPIDGradual(45, 45);
  lockOff();
  task::sleep(500);

  // INTAKE RED LINE OF 3 /////////////////
  task rbvhsvhdv(resetSlingAsync);
  forwardPID(-36, 30);
  task::sleep(3000);
  turnPID(-45,30);
  setIntake(100);
  forwardPIDGradual(64, 40);
  task::sleep(400);
  forwardInches(-3.5, 10);

  // BLUE GOAL #2 //////////// 
  turnPID(45, 26);
  stopIntake();
  forwardInches(9, 12);
  task::sleep(100);
  lockOff();
  task::sleep(100);
  forwardInches(-5, 12);
  turnPID(-55, 30);
  task::sleep(100);

  // LR ROLLER ///////////////
  forwardPIDGradual(56, 55);
  turnPID(-180, 25);
  forwardInchesTimed(-20, 25, 3500);
  roller(500, 100);
  forwardPID(25, 30);

  // RB ROLLER ///////////////
  turnPID(-270, 25);
  forwardInchesTimed(-32, 30, 6000);
  roller(500, 100);

  // ENDGAME ////////////////
  forwardInches(29, 26);
  turnPID(-270+46, 23);
  forwardInches(-30, 26);
  manspread(); // usually hits 20-24 tiles
}


void skillsEndgame() {
  forwardInches(3, 15);
  task::sleep(1000);
  manspread();
}

void noAuto() {}

void lowGoal() {
  task::sleep(1000);
  lockOff();
}


bool intakeOn = false;

void toggleIntake() {
  intakeOn = !intakeOn;
}


void pidTester()
{
  //forwardPID(10, 20);
  turnPIDSmall(8, 25);
  //turnPID(90, 25);
  //turnPID(-45, 25);
  //turnPID(180, 35);
}


// DRIVER ///////////////////////////////////////////////////////////////////
void usercontrol(void) {
  // set up controller callbacks beforre the code starts
  // these run the function in a new task so it doesn't interrupt the rest of driver code
  controllerPrim.ButtonDown.pressed(toggleLock);
  controllerPrim.ButtonUp.pressed(manspread); // deploy endgame mech
  controllerPrim.ButtonB.pressed(launchResetSling);
  controllerPrim.ButtonA.pressed(togglePto);
  controllerPrim.ButtonR1.pressed(toggleIntake);

  L1BASE.setStopping(brakeType::coast);
  L2BASE.setStopping(brakeType::coast);
  L3BASE.setStopping(brakeType::coast);

  R1BASE.setStopping(brakeType::coast);
  R2BASE.setStopping(brakeType::coast);
  R3BASE.setStopping(brakeType::coast);

  INTAKE.setStopping(brakeType::coast);
  XBOW.setStopping(brakeType::coast);

  resettingSling = false;

  // User control code here, inside the loop
  while (1 == 1) {
    L1BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    L2BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    L3BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    R1BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    R2BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    R3BASE.spin(fwd, controllerPrim.Axis2.value(), pct);

    // sling reload buttons
    if (controllerPrim.ButtonL1.pressing()) { // unwind
      XBOW.spin(reverse, 100, pct);
      INTAKE.spin(reverse, 100, pct); // have intake help pull back
      resettingSling = false;
      //intakeOn = false;

    } else if (controllerPrim.ButtonL2.pressing()) { // pullback
      XBOW.spin(forward, 100, pct);
      INTAKE.spin(forward, 100, pct); // have intake help pull back
      resettingSling = false;
      //intakeOn = false;

    } else if (!resettingSling){
      XBOW.stop();
      if (intakeOn) {
        setIntake(100);
      }
      else {
        stopIntake();
      }
      // intake controls
        /*if (controllerPrim.ButtonR2.pressing()) {
          INTAKE.spin(forward, 100, pct); // outtake just in case
        } else if (controllerPrim.ButtonR1.pressing()) {
          INTAKE.spin(reverse, 100, pct); // intake
        } else {
          INTAKE.stop();
        }*/
    }
  
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(leftFull);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  // A lot of asyncronous tasks separate from the auton and driver task occur here
  while (true) {
    // Run these independently of auton and driver tasks

    task::sleep(10);
    controllerPrim.Screen.setCursor(1, 1);
    task::sleep(10);
    controllerPrim.Screen.print("%.2f    ", getRotationDeg());

    /*if (Brain.Screen.pressing()) {
      //INERTIAL.calibrate();
    }*/

    // Show the debug values
    Brain.Screen.setPenColor(white); // Set text color to white
    //Brain.Screen.setFillColor(black);

    Brain.Screen.setFont(fontType::mono30);
    Brain.Screen.printAt(10, 30, "Rot: %.2f deg  ", getRotationDeg());
    Brain.Screen.printAt(10, 60, "Dist: %.2f in  ", getTotalDistance());

    Brain.Screen.setFont(fontType::mono20);

    Brain.Screen.printAt(10, 100,"LK %d  EG: %d  PTO: %d  3S: %d",
      (int)LOCK.value(),(int)ENDGAME.value(),(int)PTO.value(),(int)MUNCHER.value());

    Brain.Screen.printAt(10, 120,"resetting: %d  bump: %d", (int)resettingSling, (int)CATALIMIT.pressing());
    Brain.Screen.printAt(10, 140,"windup: %d deg  %.3f rev   ", (int)XBOW.rotation(deg), XBOW.rotation(rev));

    task::sleep(20);
  }
}