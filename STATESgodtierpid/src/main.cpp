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

    
    int intakeDelaySafe()
    {
      waitUntil(intakeSafe);
      task::sleep(50);
      setIntake(100);
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



int LFintakeDelay()
{
  task::sleep(250);
  setIntake(80);
  munchOn();
  task::sleep(1000);
  munchOff();
  return 0;
}
        

void leftFull()
{
            //task k(resetSlingAsync);
            //curvePID(9, 25, 2.375);
            //curvePID(14, 40, 2);
            //task::sleep(99999);

            /*etBase(-10);
            task::sleep(300);
            //setBase(-2);*/
            //task::sleep(250);

// 2-25
/*
  setIntake(100);
  forwardPID(4.5, 10);
  
  turnPID(-40, 25);
  task::sleep(150);
  stopIntake();
  lockOff();
  task::sleep(100);
  forwardInchesTimed(-10, 35, 2000);
  roller(360, 100);
  forwardPID(5, 15);
  turnPID(-91, 26);
  */



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
  
  task fsldj(intakeDelaySafe);
  forwardPID(50,40);
  

  turnPID(67, 27);
  /*forwardPID(-4,18);
  task::sleep(100);*/
  task::sleep(75);
  lockOff();

  //curvePID(17.45, 25, 2.375);


      // turnPIDSmall(6,12);
      // lockOff();

      // task a(shootWhileMove);
      // forwardPID(12, 100);
      // holdBase();

  //turnPID(-5,12);

  // setIntake(100);
  // forwardPID(6, 16);
  // turnPIDSmall(4, 12);

  // stopIntake();
  // task::sleep(100);
  // lockOff();
  // task::sleep(75);

  // task b(resetSlingAsync);
  // turnPID(-58.5,20); // trun to 3 stack MESSES UP GOES TOO RIGHT SOMETIMES
  // task::sleep(100);
  // munchOn();

  // task c(intakeDelayMunch);
  // forwardPID(45,33); // to 3 stack
  // task::sleep(50);
  // turnPID(26.5, 32);
  // forwardPID(-7.5, 28);
  // stopIntake();
  // task::sleep(75);
  // lockOff(); // shot #2
  
  // task d(resetSlingAsync);
  // task::sleep(75);
  // //curvePID(9, 26, 2.375);
  // curvePID(17.45, 25, 2.375);
  // task::sleep(50);
  // task e(intakeDelaySafe);

  // forwardPID(85, 75);
  // turnL(250, 30);
  // forwardInches(-5, 50);

          //munchOff();

          /*
            task a(shootWhileMove);
            forwardPID(12, 100);
            holdBase();
            forwardInches(-15, 30);
            roller(360, 100);

            task b(resetSlingAsync);
            munchOn();
            forwardPID(5, 20);
            turnPID(-55, 20);
            waitUntil(!resettingSling);
            setIntake(100);
            task c(downMunch);
            forwardPID(60, 30);
            munchOff();*/

}
          int intakeDelayMunchL9()
          {
            waitUntil(intakeSafe);
            //task::sleep(1750);
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
  forwardInches(3, 10);
  turnPID(34, 27);
  forwardPIDGradual(18, 30, 1.25);
  task::sleep(100);
  lockOff();
  task::sleep(100);
  task gssdg(resetSlingAsync);
  turnPID(86, 26);
  task ojwie(intakeDelayMunchL9);
  forwardPID(13, 30);
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



void egSkills() {
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
  
  //forwardPID(10, 50);
  //forwardPID(20, 60);
  //forwardPIDStraight(20, 60, 0);
  turnPID(80, 33);
  //turnPID(90, 40);
  forwardPID(-1, 50);
  /*forwardPID(1, 50);*/
  //turnPIDSmall(8, 25);
  //turnPID(90, 25);
  //turnPID(180, 100);
  //turnPID(180, 35);
}


void curveTest()
{
  forwardPID(10, 18);
  //curvePID(9*2, 25, 2.375);
  //forwardPIDStraight(20*2, 40, -5);
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
  Competition.autonomous(pidTester);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  // A lot of asyncronous tasks separate from the auton and driver task occur here
  while (true) {
    // Run these independently of auton and driver tasks

    task::sleep(10);
    controllerPrim.Screen.setCursor(1, 1);
    printf("%f \n", getRotationDeg());
    task::sleep(10);
    controllerPrim.Screen.print("%.2f    %.2f", getRotationDeg(), XBOW.rotation(deg));
    

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