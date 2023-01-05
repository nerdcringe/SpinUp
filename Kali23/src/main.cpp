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
void lockOn() { LOCK.set(1); }
void lockOff() { LOCK.set(0); }


void manspread() {
  ENDGAME.set(!ENDGAME.value());// deploy endgame mech
  }


void setIntake(double speed){ INTAKE.spin(fwd,speed,velocityUnits::pct); }
void stopIntake() { INTAKE.stop(coast); }


bool resettingSling = false; // set to false to cancel reset

void resetSling() {
  resettingSling = true; // reset can be canceled with manual buttons in driver control

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
  XBOW.rotateTo(0, deg, 100, velocityUnits::pct); // xbow unwinds by itself without intake
  resettingSling = false;
}


/*
void shootSling(){
  lockOff();
  wait(500, msec);
  resetSling();
}

int shootSlingAsync() {
  shootSling();
  return 0;
}*/

void lerpTester(double inches, int startSpeed, int endSpeed)
{
  resetTotalDistance();
  double targetDistance = inches;
  //double error = targetDistance;

  while(true)
  {
    //double error = targetDistance - getTotalDistance();
    //double errorRatio = error/ targetDistance;

    double t = getTotalDistance()/targetDistance;
    double speed = startSpeed * t + endSpeed * (1 - t);

    if( t >= 1){
      break;
    }

    setLeftBase(speed);
    setRightBase(speed);
  }
  stopBase();
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


void leftRoller() {
  forwardInchesTimed(-3,45, 750);
  forwardInchesTimed(2, 30, 1000);
  
  task::sleep(500);
  forwardInchesTimed(-3, 45, 750);
  forwardInchesTimed(3, 30, 1000);
  holdBase();
  task::sleep(500);
  
  forwardInchesTimed(-6, 45, 750);
  forwardInchesTimed(3, 25, 1000);
  task::sleep(500);
  holdBase();
  
  forwardInchesTimed(-6, 45, 750);
  forwardInchesTimed(3, 25, 1000);
  task::sleep(500);
  holdBase();

  forwardInchesTimed(-6, 45, 750);
  forwardInchesTimed(3, 25, 1000);
}

void rollerRight() {
  forwardPID(18, 30);
  turnPID(-90, 25);
  forwardInchesTimed(7, 30, 2000);

}

void rightHalf() {
  
  forwardPID(-25, 30);
  gyroTurn(-88.5, 24);
  forwardInchesTimed(-13, 32, 750);
  
  forwardInchesTimed(2.5, 30, 1000);

  forwardInchesTimed(-3.5, 32, 750);
  forwardInchesTimed(2.5, 30, 1000);
  
  
  forwardInchesTimed(-3.5, 32, 750);
  forwardInchesTimed(2.5, 30, 1000);

  forwardInchesTimed(-3.5, 32, 750);
  forwardInchesTimed(2.5, 30, 1000);

  /*
  forwardInchesTimed(-3.5, 15, 750);
  forwardInchesTimed(3, 20, 3000);
  //task b(shootSling());
  turnPID(45, 25); // turn to midfield
  task::sleep(400);
  forwardPID(32.5, 40);
  turnPID(-45, 25); // turn to goal
  task::sleep(400);
  shootSling();*/

}

void rightDiscs(){
  forwardInches(10, 12);

  turnPID(44.5, 25);
  //forwardPID(70,50);
  task::sleep(50);
  forwardPIDGradual(52,57);
  turnPID(-45,23); // turn to shoot
  task::sleep(250);

  lockOff();
  task::sleep(500);
}

void skills() {
  forwardInches(3, 15);
  task::sleep(1000);
  manspread();
}

void leftHalf(){
  forwardInchesTimed(-3.5, 28, 350);
  forwardInchesTimed(2.25, 30, 500);
  
  forwardInchesTimed(-3.5, 28, 350);
  forwardInchesTimed(2.25, 30, 500);

  forwardInchesTimed(-3.5, 28, 350);
  forwardInchesTimed(2.25, 30, 500);

  forwardInchesTimed(-3.5, 28, 350);
  forwardInchesTimed(2.25, 30, 500);

  forwardInches(1.5, 20);
  forwardInches(1, 10);

  turnPID(-45.5, 25);
  //forwardPID(70,50);
  task::sleep(50);
  forwardPIDGradual(70,57);
  turnPID(45,23); // turn to shoot
  task::sleep(50);

  lockOff();
  task::sleep(250);
}

void leftFull(){
  
  /*turnPIDDist(12, 22); // inertial sensor angle doesnt work for some reason so have to turn by distance
  task::sleep(99999);*/
  /*  turnPID(-46.5, 25);
  turnPID(45,23); // turn to shoot
    turnPID(-47, 24);*/
    //task::sleep(99999);
  forwardInchesTimed(-3.5, 28, 350);
  forwardInchesTimed(2.25, 30, 500);
  
  forwardInchesTimed(-3.5, 28, 350);
  forwardInchesTimed(2.25, 30, 500);

  forwardInchesTimed(-3.5, 28, 350);
  forwardInchesTimed(2.25, 30, 500);

  forwardInchesTimed(-3.5, 28, 350);
  forwardInchesTimed(2.25, 30, 500);

  forwardInches(1.5, 20);
  forwardInches(1, 10);

  turnPID(-46, 25);
  //forwardPID(70,50);
  task::sleep(50);
  forwardPIDGradual(70,57);
  turnPID(45,23); // turn to shoot
  task::sleep(50);
  /*task v(shootSlingAsync);
  task::sleep(1800);*/
  
  lockOff();
  task::sleep(250);
  turnPIDDist(-8, 22); // inertial sensor angle doesnt work for some reason so have to turn by distance
  //turnPID(-44.3, 24);

  task::sleep(100);
  forwardPIDGradual(68.5,57);
  
  //task::sleep(100);
  //turnPID(90, 25);
  turnPIDDist(12.25, 22); // inertial sensor angle doesnt work for some reason so have to turn by distance
  //turnPIDDist(-8, 22);

  forwardInchesTimed(-10, 32, 450);
  
  forwardInchesTimed(2.5, 30, 1000);

  forwardInchesTimed(-3.5, 32, 750);
  forwardInchesTimed(2.5, 30, 1000);
  
  forwardInchesTimed(-3.5, 32, 750);
  forwardInchesTimed(2.5, 30, 1000);

  forwardInchesTimed(-3.5, 32, 750);
  forwardInchesTimed(2.5, 30, 1000);

  //forwardInchesTimed(-3.5, 15, 750);
  //forwardInchesTimed(2, 30, 2000);
  
  //forwardInchesTimed(-3.5, 15, 750);
  //forwardInchesTimed(2, 30, 2000);


}

void noAuto() {}

void lowGoal() {
  task::sleep(1000);
  lockOff();
}

void pidTest(){
  //forwardPID(30, 40); // aight
  //wait(3, sec);
  //turnPID(45, 20);
  //turnPID(90, 25);
  
    //turnPID(90,25);

    //task d(driveStraight);
    double a = 0;
    repeat(8)
    {
      forwardPID(24, 35);
      wait(100, msec);

      a += 90;
      turnPID(a, 25);
      wait(100, msec);
    }
    
}


// DRIVER ///////////////////////////////////////////////////////////////////
void usercontrol(void) {
  // set up controller callbacks beforre the code starts
  // these run the function in a new task so it doesn't interrupt the rest of driver code
  controllerPrim.ButtonB.pressed(toggleLock);
  controllerPrim.ButtonX.pressed(manspread); // deploy endgame mech
  //controllerPrim.ButtonLeft.pressed(resetSling);

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


    // sling reload buttons
    if (controllerPrim.ButtonL1.pressing()) { // unwind
      XBOW.spin(reverse, 100, pct);
      //INTAKE.spin(forward, 100, pct);

    } else if (controllerPrim.ButtonL2.pressing()) { // pullback
      XBOW.spin(forward, 100, pct);
      INTAKE.spin(forward, 100, pct); // have intake help pull back

    } else {
      XBOW.stop();
      // intake controls
      if (controllerPrim.ButtonR2.pressing()) {
        INTAKE.spin(forward, 100, pct); // intake
      } else if (controllerPrim.ButtonR1.pressing()) {
        INTAKE.spin(reverse, 100, pct); // outtake just in case
      } else {
        INTAKE.stop();
      }
    }

    /*
    */
  
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}



void values() {
  Brain.Screen.setPenColor(white); // Set text color to white

  //Brain.Screen.printAt(0, 20, "%d 1575Y YOYO %d   ", getRotationDeg(), getRotationDeg());

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
  Competition.autonomous(rightDiscs);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  // A lot of asyncronous tasks separate from the auton and driver task occur here
  while (true) {
    //controllerPrim.Screen.clearScreen();
    task::sleep(10);
    controllerPrim.Screen.setCursor(1, 1);
    task::sleep(10);
    controllerPrim.Screen.print("%.2f    ", getRotationDeg());

    // Run these independently of auton and driver tasks
    if (Brain.Screen.pressing()) {
      //INERTIAL.calibrate();
    }

    // Show the debug values and the odometry display
    values();
    task::sleep(20);
  }
}