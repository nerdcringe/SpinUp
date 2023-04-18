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

void setIntake(double speed=100){
  LUPPER.spin(reverse,speed,velocityUnits::pct);
  RUPPER.spin(reverse,speed,velocityUnits::pct);
}

void stopIntake() {
  LUPPER.stop(coast);
  RUPPER.stop(coast);
}


void setRoller(double speed=100) {
  LUPPER.spin(fwd,speed,velocityUnits::pct);
  RUPPER.spin(fwd,speed,velocityUnits::pct);
}
void rollerFor(double dist, double speed) {
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
void munchUp() { MUNCH.set(1);}
void munchDown() { MUNCH.set(0);}

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
int pullbackAutoIntake(){
  pullbackDriver();
  setIntake(100);
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

void leftHalf() {
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

void right3Stack() {

  turnPID(-15, 30, 1500);
  //lockOff();
  task::sleep(250);
  //turnPID(-45, 25, 1500);
  gyroTurn(-45, 25);
  forwardPIDStraight(17, 25, -45);
  forwardPID(-5, 25);
  turnPID(-15, 25, 2000);
      //task bssfe(pullbackAuto);
}



int rollerDelayRH()
{
  task::sleep(1000);
  waitUntil(getTotalDistance() < -58.5);
  setRoller(100);
  task::sleep(1500);
  return 0;
}

void right5() {
  turnPIDSmall(-16, 30, 600);
  lockOff();
  task::sleep(50);
  //task sdsjgj (pullbackAutoIntake);
  
  task::sleep(150);
          //setIntake(100);
  curveL(2.6, 9*1.6, 17.5*1.6);
  forwardPIDStraight(30, 34, 45);
      //turnPID(-37, 20);
  turnPID(-43, 25, 1000);
  forwardPIDStraight(9, 30, -43, 1000);

  task::sleep(50);
  lockOff();
  task::sleep(150);
  //task gseup(pullbackAutoIntake);
  forwardInches(-7, 20);
  gyroTurn(45, 55);
  //setRoller(100);
  task vawoit(rollerDelayRH);
  forwardPIDStraight(-59.75, 50, 45);
  holdBase();
  task::sleep(500);
  stopBase();
}



      int shootWhileMoveR6() {
        task::sleep(250);
        lockOff();
        return 0;
      }


int rollerDelayR6()
{
  task::sleep(1000);
  waitUntil(getTotalDistance() < -40);
  setRoller(100);
  task::sleep(1500);
  return 0;
}
void right6() {
  
  /*gyroTurn(-45-31, 62);
  task::sleep(99999);*/

        /*
        setIntake(100);
        forwardPID(26, 34);
        turnPID(-26,26);
        */
  setIntake(100);
  forwardPID(16, 34);
  //turnPID(-24.5,26);
  gyroTurn(-24.5, 25);
  
  //task aoiewvtu(shootWhileMoveR6);
  forwardPIDStraight(10, 50, -24.5);
  
  //task randomname(pullbackAutoIntake);
  wait(0.4, sec);
        /*
        task::sleep(100);
        lockOff();
        task::sleep(100);*/
  forwardInches(-6, 20);
  //forwardPID(-4, 30);
  turnPID(45, 30); // towards line of 3
  forwardPID(30,35);
  forwardPID(-15,55);
  gyroTurn(-31, 62);
  forwardInches(7,20); // pick up disc on line
  wait(0.85, sec);
  lockOff();

  curveL(-3.25, -25*2, -1*1.2);
  stopIntake();
  task vawoit(rollerDelayRH);
  forwardPIDStraight(-49, 80, 41);
  
}


void rightBarrier() {  
  turnPIDSmall(-16, 30, 600);
  lockOff();
  task::sleep(50);
  //task sdsjgj (pullbackAutoIntake);
  
  task::sleep(150);
          //setIntake(100);
  curveL(2.6, 9*1.6, 17.5*1.6);
  forwardPIDStraight(30, 34, 45);
      //turnPID(-37, 20);
  turnPID(-43, 25, 1000);
  forwardPIDStraight(9, 30, -43, 1000);

  task::sleep(50);
  lockOff();
  task::sleep(150);
  //task gseup(pullbackAutoIntake);
  curveR(-5.6, 2*0.6, -60*0.6);

        //curveL(8, 18*1.5, 20*1.5);
        //forwardInches(15, 20);
  setIntake(100);
  curveL(4, 19.5, 20);
  //setIntake(100);
  curveL(7.25, 16*1.25, 20*1.25);
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

      // only do intake controls if not pulling back
      } else if (intakeSafe) {
        // intake automation
        if (intakeOn) {
          LUPPER.spin(reverse, 100, pct);
          RUPPER.spin(reverse, 100, pct);
           
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
  Competition.autonomous(right6);
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
  int i = 0;
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);

    // write every other frame so its easier to read
    if (i % 2 == 0) {
      controllerPrim.Screen.setCursor(1, 1);
      //printf("%f \n", getRotationDeg());
      controllerPrim.Screen.print("%.2f  ", getRotationDeg());
    }
    i = i + 1;
    
    // Show the debug values
    Brain.Screen.setFont(fontType::mono30);
    Brain.Screen.printAt(10, 30, "Rot: %.2f deg  ", getRotationDeg());
    Brain.Screen.printAt(10, 60, "Dist: %.2f in  ", getTotalDistance());
    Brain.Screen.setFont(fontType::mono20);
    Brain.Screen.printAt(10, 100-10,"BASEPTO  %d B", BASEPTO.value());
    Brain.Screen.printAt(10, 120-10,"LOCK     %d R", LOCK.value());
    Brain.Screen.printAt(10, 140-10,"ENDGME   %d U", ENDGAME1.value());
    Brain.Screen.printAt(10, 160-10,"BANDPTO  %d L", BANDPTO.value());
    Brain.Screen.printAt(10, 180-10,"MUNCH    %d X", MUNCH.value());
    Brain.Screen.printAt(10, 200-10,"PULBACK  %d D", !intakeSafe);
    Brain.Screen.printAt(10, 220-10,"INTAKE   %d R1", intakeOn);
    Brain.Screen.printAt(10, 240-10,"LIMIT    %d ", LIMIT.value());
  }
}
