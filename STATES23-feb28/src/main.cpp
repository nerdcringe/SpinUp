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
bool intakeSafe = true;




// launches and resets for driver
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

  resettingSling = false;
}


// reset in auto, cant move at the same time
void resetSling() {
  resettingSling = true; // reset can be canceled with manual buttons in driver control
  intakeSafe = false;
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
  intakeSafe = true;

  // unwind string to original position
  //INTAKE.spin(reverse, 100, pct);
  XBOW.rotateTo(0, deg, 100, velocityUnits::pct); // xbow unwinds by itself without intake
  //INTAKE.stop();
  resettingSling = false;
}


//  call in task for auto so you can move at the same time
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

int i = 0;
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  
  printf("Callibrating %d \n", i);
  i++;
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


    int shootWhileMove()
    {
      task::sleep(400);
      lockOff();
      return 0;
    }

    
    int intakeDelaySafe80()
    {
      waitUntil(intakeSafe);
      task::sleep(50);
      setIntake(80);
      return 0;
    }

    int intakeDelayMunch()
    {
      waitUntil(intakeSafe);
      //task::sleep(1750);
      setIntake(100);
      task::sleep(600);
      munchOff();
      return 0;
    }

    int munchOffDelay()
    {
      task::sleep(750);
      munchOff();
      return 0;
    }


/*
int LFintakeDelay()
{
  task::sleep(250);
  setIntake(80);
  munchOn();
  task::sleep(1000);
  munchOff();
  return 0;
}*/


      int intakeDelayMunchLF()
      {
        waitUntil(intakeSafe);
        task::sleep(100);
        setIntake(100);
        task::sleep(600);
        munchOff();
        return 0;
      }


/*
// PICK UP CLOSE 1 SHOOT AND PICKUP 3 STACK AND SHOOT
void leftFull()
{
    setIntake(100);
  forwardPID(5, 35);
  turnPID(-34.5, 28);
  
  //forwardPID(18, 30);
  task::sleep(100);
  lockOff();
  task::sleep(100);
  
  turnPID(-10, 28);
  forwardPID(-6, 30, 2000);
  //holdBase();
  roller(250, 100);
  setBase(4);
  task::sleep(100);
  task gdskfj(resetSlingAsync);
  forwardPID(4, 18);
  //turnPIDSmall(-15, 24);
  turnPID(-95, 25);
  //munchOn();
  task fsldj(intakeDelaySafe80);
  forwardPIDStraight(49,39, -95);
  //forwardPID(50,40);

  turnPID(-22, 27);
  lockOff();
  //gyroTurn(-45,34);
}
*/

/*
void leftFull()
{

  forwardInchesTimed(-0.75, 5, 1000);
  holdBase();

  roller(300, 100);
  task::sleep(50);

  //forwardPID(4,20);
  
  curvePID(12, 20, 2.5);
  //turnPID(-53,20);
  setIntake(80);
  //munchOn();
  
  //task esg(LFintakeDelay);
  // /forwardPID(45, 45);//45
  
  //setIntake(100);
  task fsfl(munchOffDelay);
  forwardPIDStraight(48, 45, -55);

  turnPID(36, 30);
  stopIntake();
  
  forwardPID(-14, 18);
  
  task::sleep(75);
  lockOff();
  task::sleep(75);
  
  task fsldfks(resetSlingAsync);
  
  
  forwardPID(14,17);
  turnPID(-44, 28);
  
  task fsldj(intakeDelaySafe80);
  forwardPID(50,40);
  

  turnPID(67, 27);
  //forwardPID(-4,18);
  //task::sleep(100);
  task::sleep(75);
  lockOff();

  //curvePID(17.45, 25, 2.375);


      // turnPIDSmall(6,12);
      // lockOff();

      // task a(shootWhileMove);
      // forwardPID(12, 100);
      // holdBase();

  //turnPID(-5,12);
}*/


void leftFull()
{

  forwardInchesTimed(-0.75, 5, 1000);
  holdBase();

  roller(300, 100);
  task::sleep(50);

  //forwardPID(4,20);
  
  curvePID(12, 20, 3.5);
  //turnPID(-53,20);
  setIntake(100);//80
  //munchOn();
  
  //task esg(LFintakeDelay);
  // /forwardPID(45, 45);//45
  
  //setIntake(100);
  
  //task fsfl(munchOffDelay);
  forwardPIDStraight(49, 37, -55);

  //turnPID(33, 34);
  gyroTurn(33, 35);
  stopIntake();
  forwardPIDStraight(-10, 18, 38);
  
  task::sleep(75);
  lockOff();
  task::sleep(50);
  
  task fsldfks(resetSlingAsync);
  
  task::sleep(800);
  
  curvePID(16, 13, 2.5);

  //turnPID(-44, 28);
  //task::sleep(300);
  //setIntake(80);

  task fsldj(intakeDelaySafe80);
  
  forwardPIDStraight(49,45, -45);
  //forwardPID(50,40);

  /*turnPID(67, 27);
  lockOff();
  gyroTurn(-45,34);
  forwardPIDStraight(25,45, -45);*/
  
  setIntake(100);
  turnPID(140, 30);
  forwardPIDStraight(-30,75, 140);


}
        

void left6()
{

  forwardInchesTimed(-0.75, 5, 1000);
  holdBase();

  roller(300, 100);
  task::sleep(50);

  //forwardPID(4,20);
  
  curvePID(12, 20, 3.5);
  //turnPID(-53,20);
  setIntake(100);//80
  //munchOn();
  
  //task esg(LFintakeDelay);
  // /forwardPID(45, 45);//45
  
  //setIntake(100);
  
  //task fsfl(munchOffDelay);
  forwardPIDStraight(49, 37, -55);

  //turnPID(33, 34);
  gyroTurn(33, 35);
  stopIntake();
  forwardPIDStraight(-10, 18, 34);
  
  task::sleep(75);
  lockOff();
  task::sleep(50);
  
  task fsldfks(resetSlingAsync);
  
  task::sleep(1000);
  
  curvePID(16, 13, 2.5);

  //turnPID(-44, 28);
  //task::sleep(300);
  //setIntake(80);

  task fsldj(intakeDelaySafe80);
  
  forwardPIDStraight(49,39, -45);
  //forwardPID(50,40);

  turnPID(67, 27);
  lockOff();
  gyroTurn(-45,34);
  forwardPIDStraight(25,45, -45);

}
     
     
      int intakeDelayMunchL9()
      {
        waitUntil(intakeSafe);
        task::sleep(100);
        setIntake(100);
        task::sleep(600);
        munchOff();
        return 0;
      }

      

void left9()
{
  /*forwardInchesTimed(-1.5, 15, 1000);
  holdBase();

  roller(300, 100);
  task::sleep(50);

  forwardPID(4,20);
  turnPID(-20,20);*/
  setIntake(100);
  forwardPID(5, 10);
  turnPID(-34.5, 28);
  
  //forwardPID(18, 30);
  task::sleep(100);
  lockOff();
  task::sleep(100);
  
  turnPID(-45, 28);
  forwardPID(-8, 25, 2000);
  //holdBase();
  roller(250, 100);
  setBase(4);
  task::sleep(100);
  task gdskfj(resetSlingAsync);
  forwardPID(6, 18);
  //turnPIDSmall(-15, 24);
  
  munchOn();
  task kjhk(intakeDelayMunchL9);
  //forwardPIDStraight(35, 30, -55);
  //turnPID(-90,25);
  //forwardPID(,30);
  
  curvePID(8, 6, 3.5);
  //forwardPIDStraight(48, 45, -90);
  
  //turnPID(-70, 28);
  /*task gssdg(resetSlingAsync);
  turnPID(86, 26);
  task ojwie(intakeDelayMunchL9);
  forwardPID(13, 30);*/
}




    
    int intakeDelaySafeRH()
    {
      waitUntil(intakeSafe);
      task::sleep(50);
      setIntake(80);
      return 0;
    }


void rightHalf()
{
  
 /* gyroTurn(45, 16);
  
  task::sleep(999099);*/
  XBOW.setRotation(0, deg);
  setIntake(100);
  forwardPID(13, 34);
  turnPID(-24,26);
  
  task::sleep(100);
  lockOff();
  task::sleep(150);
  
  turnPID(0,26);
  task randomname(resetSlingAsync);
 
  forwardPID(-15, 45);
  //INERTIAL.setRotation(0, deg);
  gyroTurn(-49, 20);
  task sgdsd(intakeDelaySafeRH);
  munchOn();
  //forwardPIDStraight(28.5, 23, -50);
  forwardPIDStraight(28.5, 30, -50);
  //forwardInches(5, 10);
  munchOff();
  task::sleep(400);
  forwardPID(-10, 34);
  turnPID(-14, 26);

  task::sleep(100);
  lockOff();
  //turnPID(50, 25);
  
  task::sleep(250);
  //INERTIAL.setRotation(-14, deg);
  gyroTurn(50, 19);
  
  setIntake(100);
  forwardPIDStraight(-24, 60, 45);
  //forwardInches(-14, 75);
  //roller(360, 100);
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



void badSkills() {
  forwardInches(3, 15);
  task::sleep(1000);
  manspread(); // endgame
}

void noAuto() {}



// SKILLS //
// Roboticon & states
void skills() {
  // LB ROLLER ////////////////
  setBase(-10); // set the speed to -10% for .35 seconds
  task::sleep(350);
  setBase(-2); // set the speed to -2%
  roller(360, 100); // 360 degrees at 100% speed
  //wait(0.15, sec);
  task jsdhgkj(intakeDelay); // start the intake in a separate task
  forwardInches(26, 22); // distance, then speed
  task::sleep(500); // wait 500 milliseconds (.5 seconds)

  // RR ROLLER ///////////////
  turnPID(-90, 37); // turn to -90 degrees (relative to starting orientation of robot) at 37% speed
  task gsohgo(stopIntakeDelay);
  forwardInchesTimed(-25, 45, 1000); // distance, speed, then timeout (exit after a second in case it gets stuck on wall)
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




void pidTester()
{
  
  //forwardPID(10, 50);
  //forwardPID(20, 60);
  //forwardPIDStraight(20, 60, 0);
  //turnPID(80, 33);
  turnPID(90, 45);
  forwardPID(-1, 50);
  /*forwardPID(1, 50);*/
  //turnPIDSmall(8, 25);
  //turnPID(90, 25);
  //turnPID(180, 100);
  //turnPID(180, 35);
  
  //forwardPID(10, 18);
  //curvePID(9*2, 25, 2.375);
  //forwardPIDStraight(20*2, 40, -5);
}



bool intakeOn = false;

void toggleIntake() {
  intakeOn = !intakeOn;
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
  controllerPrim.ButtonX.pressed(toggleMunch);

  L1BASE.setStopping(brakeType::coast);
  L2BASE.setStopping(brakeType::coast);
  L3BASE.setStopping(brakeType::coast);

  R1BASE.setStopping(brakeType::coast);
  R2BASE.setStopping(brakeType::coast);
  R3BASE.setStopping(brakeType::coast);

  INTAKE.setStopping(brakeType::coast);
  XBOW.setStopping(brakeType::coast);

  resettingSling = false;
  intakeSafe = false;

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
      // old intake controls
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
    //printf("%f \n", getRotationDeg());
    task::sleep(10);
    controllerPrim.Screen.print("%.2f    %d", getRotationDeg(), (int)XBOW.rotation(deg));
    

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