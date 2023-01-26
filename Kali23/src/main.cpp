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
  XBOW.rotateTo(0, deg, 100, velocityUnits::pct); // xbow unwinds by itself without intake
  resettingSling = false;
}

//  call in task for auto
int resetSlingAsync(){
  resetSling();
  return 0;
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
      double t = getTotalDistance()/targetDistance; // ratio of distance travelled (starts at 0 and ends at 1)
      double speed = startSpeed * t + endSpeed * (1 - t); // Gradually transition between startSpeed and endSpeed

      if( t >= 1){
        break;
      }

      setLeftBase(speed);
      setRightBase(speed);
    }
    stopBase();
  }
/*
  gyroConstant(double deg, double speed)
  {

    double tarrget = inches;
    //double error = targetDistance;

    while(true)
    {
      double t = getTotalDistance()/targetDistance; // ratio of distance travelled (starts at 0 and ends at 1)
      double speed = startSpeed * t + endSpeed * (1 - t); // Gradually transition between startSpeed and endSpeed

      if( t >= 1){
        break;
      }

      setLeftBase(speed);
      setRightBase(speed);
    }
    stopBase();
  }*/


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


// void leftFull() {
//   setBase(-10);
//   task::sleep(350);

//   stopBase();
//   roller(300, 100);
//   //lockOff();
//   //task s(resetSlingAsync);
//   forwardInches(3.25, 15);
//   //gyroTurnSlow(-38, 20);
//       //gyroTurnSlow(-47, 25);
//       turnPID(-47, 25);

//   forwardPID(70, 55);
//       //gyroTurnSlow(43, 32);
//       turnPID(43, 32);
//   //forwardInches(-6, 10);
//   toggleLock();

//   //turnR(155, 18);
//   turnPID(-37, 25);
//    forwardPIDGradual(70, 55);

//    turnL(220, 20);
//   revConst(85, 10, 9999);
//   stopBase();
//   roller(300, 100);

//   ptoOff();
//   task::sleep(100);
//   ptoOn();

//   // /*    double turnAmount = 100;
//   //   double turnSpeed = 20;
//   //   L1BASE.startRotateFor(turnAmount, deg, turnSpeed, velocityUnits::pct);
//   //   L2BASE.startRotateFor(turnAmount, deg, turnSpeed, velocityUnits::pct);
//   //   L3BASE.startRotateFor(turnAmount, deg, turnSpeed, velocityUnits::pct);
//   //   R1BASE.startRotateFor(reverse, turnAmount, deg, turnSpeed, velocityUnits::pct);
//   //   R2BASE.startRotateFor(reverse, turnAmount, deg, turnSpeed, velocityUnits::pct);
//   //   R3BASE.rotateFor(reverse, turnAmount, deg, turnSpeed, velocityUnits::pct);*/

//   // forwardInchesTimed(-6, 40, 2000);
//   // roller(300, 100);
// }



// // # 2 kali hotel w/o backup
// void leftFull() {
//   setBase(-10);
//   task::sleep(500);
//   setBase(-3);

//   //stopBase();

//   roller(300, 100);
//   //lockOff();
//   //task s(resetSlingAsync);
//   forwardInches(5, 19);
//   //gyroTurnSlow(-38, 20);
//       //gyroTurnSlow(-47, 25);
//       turnPID(-47, 26);

//   forwardPIDGradual(70, 57);
//       //gyroTurnSlow(43, 32);
//       turnPID(44, 32);
//   //forwardInches(-6, 10);
//   toggleLock();

//   //turnR(155, 18);
//   turnPID(-37, 26);
//    forwardPIDGradual(68, 56); // 65 in
//   revConst(50, 24);

//    turnL(210, 27);//220
//   revConst(200, 33, 1000);
//   stopBase();
//   roller(375, 100);

// }


// # 2 kali hotel with backup
void leftFull() {
  // ROLLER #1 ////////////////
  setBase(-10);
  task::sleep(350);
  setBase(-2);

  //stopBase();

  roller(300, 100);
  //lockOff();
  //task s(resetSlingAsync);
  forwardInches(5, 19);
  //gyroTurnSlow(-38, 20);
  //gyroTurnSlow(-47, 25);
  turnPID(-50, 26);

  forwardPIDGradual(70, 57);
  // SHOOT AT HIGH GOAL //////////
  //gyroTurnSlow(43, 32);
  turnPID(46.5, 32);
  //forwardInches(-6, 10);
  toggleLock();

  // TURN TO ROLLER #2  /////////
  //turnPID(-35, 32);
  //turnR(155, 18);
  turnPID(-34.5, 25);
   forwardPIDGradual(68, 56); // 65 in
  revConst(50, 21);

   turnL(210, 27);//220
  revConst(200, 33, 1000);
  stopBase();
  roller(375, 100);
}




// # 2 kali hotel with backup
void leftFullLong() {
  // ROLLER #1 ////////////////
  setBase(-10);
  task::sleep(350);
  setBase(-2);

  //stopBase();

  roller(300, 100);
  //lockOff();
  //task s(resetSlingAsync);
  forwardInches(5, 19);
  turnPID(11, 24);
  lockOff();

  //turnPID(-44, 27);
  turnPIDFast(-44, 23);
  task::sleep(600);
  forwardPIDGradual2(135, 65);
  turnL(240, 27);
  
  revConst(150, 20, 1500);
  setBase(-2);
  roller(375, 100);
  stopBase();


  //gyroTurnSlow(-38, 20);
  //gyroTurnSlow(-47, 25);

  
      
      // turnPID(-50, 26);

      // forwardPIDGradual(70, 57);
      // // SHOOT AT HIGH GOAL //////////
      // //gyroTurnSlow(43, 32);
      // turnPID(46.5, 32);
      // //forwardInches(-6, 10);
      // toggleLock();

      // // TURN TO ROLLER #2  /////////
      // //turnPID(-35, 32);
      // //turnR(155, 18);
      // turnPID(-34.5, 25);
      //  forwardPIDGradual(68, 56); // 65 in
      // revConst(50, 21);

      //  turnL(210, 27);//220
      // revConst(200, 33, 1000);
      // stopBase();
      // roller(375, 100);
}


int intakeDelay2()
{
  setIntake(100);
  waitUntil(getTotalDistance() > 50);
  stopIntake();
  return 0;
}

// no working
void leftIntake() {
  // ROLLER #1 ////////////////
  setBase(-10);
  task::sleep(350);
  setBase(-2);

  //stopBase();

  roller(300, 100);
  //lockOff();
  //task s(resetSlingAsync);
  forwardInches(5, 19);
  //gyroTurnSlow(-38, 20);
  //gyroTurnSlow(-47, 25);
  turnPID(-50, 26);
  
  //  setIntake(100);
  task lplekpo(intakeDelay2);

  forwardPIDGradual(70, 57);
  // SHOOT AT HIGH GOAL //////////
  //gyroTurnSlow(43, 32);
  stopIntake();
  turnPID(44, 32);
  
  

  //forwardInches(-6, 10);
  toggleLock();

  // TURN TO ROLLER #2  /////////
  //turnPID(-35, 32);
  //turnR(155, 18);
  turnPID(-34.5, 25);
   forwardPIDGradual(68, 56); // 65 in
  revConst(50, 21);

   turnL(210, 27);//220
  revConst(200, 33, 1000);
  stopBase();
  roller(375, 100);
}


void rightHalf()
{
  // ROLLER /////////////////////
  forwardInches(-20, 37);
  turnPIDFast(-90, 35);
  forwardInchesTimed(-7.5, 32, 99999);
  roller(220, 100);
  //forwardInches(10, 20);
  forwardInches(13, 25);
  task::sleep(100);
  // SHOT 1 ////////////////////////////////
  turnPIDFast(-98, 28);
  task::sleep(150);
  lockOff();
  task::sleep(150);


  // // ROLLER /////////////////////
  // forwardPID(-20, 35);
  // turnPID(-90, 28);
  // forwardInchesTimed(-7.5, 20, 99999);
  // roller(200, 100);
  // //forwardInches(10, 20);
  // forwardInches(15, 20);
  // task::sleep(500);
  // // SHOT 1 ////////////////////////////////
  // turnPID(-98, 24);
  // task::sleep(250);
  // lockOff();
  // task::sleep(250);

  // ptoOff();
  // task::sleep(100);
  // ptoOn();
  //forwardInches(5, 10);
  
  // SHOT 2 ///////////////////////////////////654
  /*turnPID(-26, 25);
  forwardInches(30, 30);
  setIntake(100);*/
  /*
  forwardInches(-12, 25);
  turnPID(-46, 26);
  setIntake(100);
  forwardPID(45, 45);
  turnPID(-135, 30);*/


}


int delayIntake() {
  task::sleep(750);
  setIntake(100);
  return 0;
}



void right5() {
  // ROLLER /////////////////////
  forwardInches(-20, 37);
  turnPIDFast(-90, 35);
  forwardInchesTimed(-7.5, 32, 4000);
  roller(220, 100);
  //forwardInches(10, 20);
  forwardInches(13, 25);
  task::sleep(100);
  // SHOT 1 ////////////////////////////////
  turnPIDFast(-98, 28);
  task::sleep(150);
  lockOff();
  task::sleep(150);
  
  task banana(resetSlingAsync);
  turnPIDFast(-110, 27);

  forwardInches(-11.5, 24);
  
  turnPID(-45, 25);
  task::sleep(100);
  
  task pineapple(delayIntake);
  forwardPIDGradual(67.5, 50);
  turnPID(-135, 25);
  stopIntake();
  lockOff();

      //   // ROLLER /////////////////////
      //   forwardInches(-20, 37);
      //   turnPIDFast(-90, 35);
      //   forwardInchesTimed(-7.5, 32, 4000);
      //   roller(220, 100);
      //   //forwardInches(10, 20);
      //   forwardInches(13, 25);
      //   task::sleep(100);
      //   // SHOT 1 ////////////////////////////////
      //   turnPIDFast(-98, 28);
      //   task::sleep(150);
      //   lockOff();
      //   task::sleep(150);
        
      //   task banana(resetSlingAsync);
      //   turnPIDFast(-110, 27);

      //   forwardInches(-11.5, 24);
        
      //   turnPID(-45, 25);
      //   task::sleep(100);
        
      //   task pineapple(delayIntake);
      //   forwardPIDGradual(67.5, 50);
      //   turnPID(-135, 25);
      //   stopIntake();
      //   lockOff();




  /*
  turnPID(-47, 25, 2500);
  task pineapple(delayIntake);
  forwardPID(65, 40);*/
  
  /*forwardInches(55, 40);
  turnPID(-127.5, 25);
  stopIntake();
  revConst(10, 30);// back up at constant speed
  lockOff();*/
}


// // AUSTINS HOUSE
// void right2Shots()
// {
//   /*XBOW.setRotation(-1400, deg);
//   resetSling();
//   task::sleep(999999);*/
//   // ROLLER /////////////////////
//   forwardPID(-16, 36);
//   turnPID(-90, 28);
//   forwardInchesTimed(-7.5, 27, 9999);
//   roller(200, 100);
//   //forwardInches(10, 20);
//   forwardInches(15, 25);
//   task::sleep(250);

//   // SHOT 1 ////////////////////////////////
//   turnPID(-99, 24);
//   task::sleep(250);
//   lockOff();
//   task::sleep(250);
//   task banana(resetSlingAsync);

//   // SHOT 2 ///////////////////////////////////
//   forwardInches(-15, 20);
//   turnPID(-45, 25);
//   task pineapple(delayIntake);
  
//   forwardInches(55, 40);
//   turnPID(-127.5, 25);
//   stopIntake();
//   revConst(10, 30);// back up at constant speed
//   lockOff();

//   ptoOff();
//   task::sleep(100);
//   ptoOn();
// }



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


void skills() {
  // ROLLER #1 ////////////////
  setBase(-10);
  task::sleep(350);
  setBase(-2);
  roller(360, 100);
  //wait(0.15, sec);
  task jsdhgkj(intakeDelay);
  forwardInches(26, 22);
  task::sleep(500);

  // ROLLER #2 /////////////
  turnPIDFast(-90, 37);
  //stopIntake();

  task gsohgo(stopIntakeDelay);

  forwardInchesTimed(-25, 45, 1000);
  setBase(-2);
  roller(360, 100);
  stopBase();
  
  forwardInches(10, 30);
  turnPID(-3, 26);
  forwardPIDGradual(45, 45);
  
  lockOff();
  task::sleep(500);

  task rbvhsvhdv(resetSlingAsync);

  forwardPID(-38,30);

task::sleep(5000);

  turnPID(-45,30);

  setIntake(100);

  forwardPIDGradual(65, 40);
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



// DRIVER ///////////////////////////////////////////////////////////////////
void usercontrol(void) {
  // set up controller callbacks beforre the code starts
  // these run the function in a new task so it doesn't interrupt the rest of driver code
  controllerPrim.ButtonB.pressed(toggleLock);
  controllerPrim.ButtonX.pressed(manspread); // deploy endgame mech
  controllerPrim.ButtonLeft.pressed(resetSling);
  controllerPrim.ButtonA.pressed(togglePto);

  L1BASE.setStopping(brakeType::coast);
  L2BASE.setStopping(brakeType::coast);
  L3BASE.setStopping(brakeType::coast);

  R1BASE.setStopping(brakeType::coast);
  R2BASE.setStopping(brakeType::coast);
  R3BASE.setStopping(brakeType::coast);

  INTAKE.setStopping(brakeType::coast);
  XBOW.setStopping(brakeType::coast);

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
      resettingSling = false;

    } else if (controllerPrim.ButtonL2.pressing()) { // pullback
      XBOW.spin(forward, 100, pct);
      INTAKE.spin(forward, 100, pct); // have intake help pull back
      resettingSling = false;

    } else if (!resettingSling){
      XBOW.stop();
      // intake controls
      if (controllerPrim.ButtonR2.pressing()) {
        INTAKE.spin(forward, 100, pct); // outtake just in case
      } else if (controllerPrim.ButtonR1.pressing()) {
        INTAKE.spin(reverse, 100, pct); // intake
      } else {
        INTAKE.stop();
      }
    }
  
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


// double Lspeed = 0;
// double Rspeed = 0;

// // DRIVER ///////////////////////////////////////////////////////////////////
// void accelReduceControl(void) {
//   // set up controller callbacks beforre the code starts
//   // these run the function in a new task so it doesn't interrupt the rest of driver code
//   controllerPrim.ButtonB.pressed(toggleLock);
//   controllerPrim.ButtonX.pressed(manspread); // deploy endgame mech
//   controllerPrim.ButtonLeft.pressed(resetSling);
//   controllerPrim.ButtonA.pressed(togglePto);

//   L1BASE.setStopping(brakeType::coast);
//   L2BASE.setStopping(brakeType::coast);
//   L3BASE.setStopping(brakeType::coast);

//   R1BASE.setStopping(brakeType::coast);
//   R2BASE.setStopping(brakeType::coast);
//   R3BASE.setStopping(brakeType::coast);

//   INTAKE.setStopping(brakeType::coast);
//   XBOW.setStopping(brakeType::coast);

//   // User control code here, inside the loop

//   double Ljoy = controllerPrim.Axis3.value();
//   double Rjoy = controllerPrim.Axis2.value();

//   if (fabs(Lspeed - Ljoy) > 5) {
//     Lspeed += Ljoy*0.01;
//   } else Lspeed = Ljoy;
  
//   if (fabs(Rspeed - Ljoy) > 5) {
//     Rspeed += Rjoy*0.01;
//   } else Lspeed = Ljoy;
  


//   while (1 == 1) {
//     L1BASE.spin(fwd, Lspeed, pct);
//     L2BASE.spin(fwd, Lspeed, pct);
//     L3BASE.spin(fwd, Lspeed, pct);
//     R1BASE.spin(fwd, Raccel, pct);
//     R2BASE.spin(fwd, Raccel, pct);
//     R3BASE.spin(fwd, Raccel, pct);

//     // sling reload buttons
//     if (controllerPrim.ButtonL1.pressing()) { // unwind
//       XBOW.spin(reverse, 100, pct);
//       resettingSling = false;

//     } else if (controllerPrim.ButtonL2.pressing()) { // pullback
//       XBOW.spin(forward, 100, pct);
//       INTAKE.spin(forward, 100, pct); // have intake help pull back
//       resettingSling = false;

//     } else if (!resettingSling){
//       XBOW.stop();
//       // intake controls
//       if (controllerPrim.ButtonR2.pressing()) {
//         INTAKE.spin(forward, 100, pct); // outtake just in case
//       } else if (controllerPrim.ButtonR1.pressing()) {
//         INTAKE.spin(reverse, 100, pct); // intake
//       } else {
//         INTAKE.stop();
//       }
//     }
  
//     wait(20, msec); // Sleep the task for a short amount of time to
//                     // prevent wasted resources.
//   }
// }



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
  Competition.autonomous(skills);
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