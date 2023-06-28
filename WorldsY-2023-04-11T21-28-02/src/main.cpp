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



///////////////////////////////////////////////////////////////////////
//                                /////////////////////////////////////
//     USER-DEFINED FUNCTIONS     /////////////////////////////////////
//                                /////////////////////////////////////
///////////////////////////////////////////////////////////////////////

void setIntake(double speed){
  LUPPER.spin(reverse,speed,velocityUnits::pct);
  RUPPER.spin(reverse,speed,velocityUnits::pct);
}

void stopIntake() {
  LUPPER.stop(coast);
  RUPPER.stop(coast);
}


void setRoller(double speed)
{
  LUPPER.spin(fwd,speed,velocityUnits::pct);
  RUPPER.spin(fwd,speed,velocityUnits::pct);
}
void roller(double dist, double speed) {
  LUPPER.spinFor(fwd,dist, rotationUnits::deg, speed,velocityUnits::pct);
  RUPPER.spinFor(fwd,dist, rotationUnits::deg, speed,velocityUnits::pct);
}




// PNEUMATIC FUNCS ////////////////////////////////////////
void lockOn() { LOCK.set(0); }
void lockOff() { LOCK.set(1); }

void toggleLock() { LOCK.set(!LOCK.value()); }
void toggleBasePTO() { BASEPTO.set(!BASEPTO.value());}
void toggleBandPTO() { BANDPTO.set(!BANDPTO.value()); }

void toggleEndgame() {
  ENDGAME1.set(!ENDGAME1.value());
  ENDGAME2.set(!ENDGAME2.value());
}

void toggleMunch() { MUNCH.set(!MUNCH.value());}

// DRIVER AUTOMATION ////////////////////////////////
bool intakeSafe = true;

void pullbackDriver() {
  lockOff();
  task::sleep(500);
  intakeSafe = false; // prevent intake controls from interfering with pullback
  RUPPER.spin(fwd, 100, pct);
  LUPPER.spin(fwd, 100, pct);

  timer resetTimer;
  resetTimer.clear();
  waitUntil(LIMIT.pressing() || resetTimer.time(sec) > 5);

  lockOn();
  task::sleep(300);
  RUPPER.stop();
  LUPPER.stop();
  intakeSafe = true;
}


int pullbackAuto(){
  pullbackDriver();
  return 0;
}



bool intakeOn = false;
void toggleIntake() { intakeOn = !intakeOn; }



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
  intakeSafe = true;
  intakeOn = false;

  printf("Callibrating \n");
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
  Brain.Screen.setFillColor(color(10, 80, 40));
  Brain.Screen.drawRectangle(0, 0, 480, 272);

  // Draw Y
  Brain.Screen.setPenColor(yellow);
  Brain.Screen.setPenWidth(40);
  int x = 385; // amount to shift to the side
  Brain.Screen.drawLine(x, 125, x, 220);
  Brain.Screen.drawLine(x, 130, x-65, 35);
  Brain.Screen.drawLine(x, 130, x+65, 35);

  Brain.Screen.setPenColor(white);

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

// leftFull - guarunteed wp, time limit
// rHalf - hard to get roller
// lHalf - more discs, people less likely to have right side

void leftFull(void) {
  
}

void leftHalf()
{
  setRoller(100);
  forwardInchesTimed(-1.5, 25, 1000);
  //holdBase();
  //roller(150, 100);
  forwardPID(4, 30);
  setIntake(100);
  task::sleep(500);
  turnPID(11, 20);
  task bssfe(pullbackAuto);

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

  intakeSafe = true;
  intakeOn = false;

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // 6m base controls
    double Lspeed = controllerPrim.Axis3.value();
    double Rspeed = controllerPrim.Axis2.value();
    L1BASE.spin(forward, Lspeed, velocityUnits::pct);
    L2BASE.spin(forward, Lspeed, velocityUnits::pct);
    L3BASE.spin(forward, Lspeed, velocityUnits::pct);
    R1BASE.spin(forward, Rspeed, velocityUnits::pct);
    R2BASE.spin(forward, Rspeed, velocityUnits::pct);
    R3BASE.spin(forward, Rspeed, velocityUnits::pct);

    if (BASEPTO.value() == 1) {

      // If PTO is in 8 motor mode, use upper motors for base
      LUPPER.spin(forward, Lspeed, velocityUnits::pct);
      RUPPER.spin(forward, Rspeed, velocityUnits::pct);

    } else {

      // If PTO is in 6 motor mode, use upper motors for intake and rollers
      if (controllerPrim.ButtonL1.pressing()) {
         // manual intaking
        LUPPER.spin(reverse, 100, pct);
        RUPPER.spin(reverse, 100, pct);
        
      } else if (controllerPrim.ButtonR2.pressing()) {
        // manual pullback
        LUPPER.spin(fwd, 100, pct);
        RUPPER.spin(fwd, 100, pct);

      } else if (intakeSafe) {
        // intake automation
        if (intakeOn) {
          LUPPER.spin(reverse, 100, pct);
          RUPPER.spin(reverse, 100, pct);

        // stop intake if  
        } else {
          LUPPER.stop();
          RUPPER.stop();
        }
      }
      
    }


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


// make highlight color either green or red depending if a variable is trrue or false
void setFillFor(bool var) {
  if (var) {
    Brain.Screen.setFillColor(color(10, 80, 40)); //green 
  } else {
    Brain.Screen.setFillColor(red);
  }
};


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // button clicks
  controllerPrim.ButtonR1.pressed(toggleIntake);
  controllerPrim.ButtonDown.pressed(pullbackDriver);

  controllerPrim.ButtonRight.pressed(toggleLock);
  controllerPrim.ButtonB.pressed(toggleBasePTO);
  controllerPrim.ButtonLeft.pressed(toggleBandPTO);
  controllerPrim.ButtonUp.pressed(toggleEndgame);
  controllerPrim.ButtonX.pressed(toggleMunch);


  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(leftHalf);
  Competition.drivercontrol(usercontrol);


  L1BASE.setStopping(brakeType::coast);
  L2BASE.setStopping(brakeType::coast);
  L3BASE.setStopping(brakeType::coast);
  LUPPER.setStopping(brakeType::coast);
  R1BASE.setStopping(brakeType::coast);
  R2BASE.setStopping(brakeType::coast);
  R3BASE.setStopping(brakeType::coast);
  RUPPER.setStopping(brakeType::coast);


  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);

    //Brain.Screen.drawRectangle(0, 270, 240, 270);
/*
    if (intakeOn) { Brain.Screen.setFillColor(blue); } else { Brain.Screen.setFillColor(orange); }
    Brain.Screen.drawRectangle(240, 270, 480, 270);*/


    // Show the debug values
    Brain.Screen.setPenColor(white); // Set text color to white
 
    Brain.Screen.setFont(fontType::mono30);
    setFillFor(true);
    Brain.Screen.printAt(10, 30, "Rot: %.2f deg  ", getRotationDeg());
    Brain.Screen.printAt(10, 60, "Dist: %.2f in  ", getTotalDistance());

    Brain.Screen.setFont(fontType::mono20);
    setFillFor(BASEPTO.value());
    Brain.Screen.printAt(10, 100,"BASEPTO %d ", BASEPTO.value());
    setFillFor(LOCK.value());
    Brain.Screen.printAt(10, 120,"LOCK %d ", LOCK.value());
    setFillFor(LIMIT.value());
    Brain.Screen.printAt(10, 140,"LIMIT %d ", LIMIT.value());
    setFillFor(ENDGAME1.value());
    Brain.Screen.printAt(10, 160,"ENDGME %d ", ENDGAME1.value());
    setFillFor(BANDPTO.value());
    Brain.Screen.printAt(10, 180,"BANDPTO %d ", BANDPTO.value());
    setFillFor(MUNCH.value());
    Brain.Screen.printAt(10, 200,"MUNCH %d ", MUNCH.value());
    setFillFor(intakeSafe);
    Brain.Screen.printAt(10, 220,"intakeSafe %d ", intakeSafe);
    setFillFor(intakeOn);
    Brain.Screen.printAt(10, 240,"intakeOn %d ", intakeOn);
/*
    Brain.Screen.printAt(10, 100,"LK %d  EG: %d  BAS: %d  BAN: %d",
      (int)LOCK.value(),(int) ENDGAME.value(),(int) BASEPTO.value(),(int)Ba.value());

    Brain.Screen.printAt(10, 120,"rst: %d  is: %d  io: %d  bmp: %d", (int)resettingSling, (int)intakeSafe, (int)intakeOn, (int)CATALIMIT.pressing());
    Brain.Screen.printAt(10, 140,"windup: %d deg  %.3f rev   ", (int)XBOW.rotation(deg), XBOW.rotation(rev));
*/
  }
}
