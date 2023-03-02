/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// This is a file that has all the libraries you plan on using in your code
// We dont really need to change this file other than add new files we want included in here
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

// This line imports all of the hardware we defined from robot-config.h and makes that available in this file
#include "robot-config.h"

// MUST have this line to allow .cpp files to reference functions from header files
#include "functions.h"
//#include "odom.h"


#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)


  
// old autos
  // void leftFull(){
  //   // have to reset these manually before every auton
  //   resetTotalDistance();
  //   resettingSling = false; // stop resetting cata if manually pressed


  //   setBase(5); // move into rollers
  //   task::sleep(300);//450
  //   holdBase();

  //   //roller(275,100);
  //   //setIntake(50);

  //           //wait(200, msec);
  //           //setIntake(0);
  //          //stopBase();
            

  //  // back up from roller and turn towards center of field
  //   forwardPID(-2.8, 35);
  //   turnPID(-45.6, 25);

  //   setIntake(100);
  //  //forwardPIDIncr(-30, 70); // go to center
  //   setIntake(0);
  //   task::sleep(50);
    
  //   //gyroTurn(-45, 20);
  //   //setIntake(100);
  //   //forwardPID(-30, 55); // go to center
  //   turnPID(44.15, 22, 1750); // turn to shoot
  //   forwardInches(-0.9, 16);
  //   setIntake(100);
  //   task::sleep(500);
  //   setIntake(0);
  //   task::sleep(100);
    
  //   R2BASE.spin(reverse, 100, pct); // shoot
  //   wait(600, msec);
  //   //task pog(catapultReset);

  //   setIntake(100);
  //   task::sleep(750);
  //   setIntake(0);
  //   //task::sleep(100);
  //   /* // shoot twice
  //   R2BASE.spin(reverse, 100, pct); // shoot
  //   wait(800, msec);
  //   task pog(catapultReset);
  //   wait(250, msec);*/

  //   forwardInches(0.75, 22);

  //   //turnPID(-44, 28);
  //   gyroTurn(-46.5, 25);
  //   forwardInches(-28, 100);
    
  //   gyroTurn(91, 30);
  //   //turnPID(90, 30);

  //   forwardInchesTimed(4.25, 50, 2500); // move into rollers
  //   setBase(5); // move into rollers
  //   task::sleep(500);
  //   holdBase();

  //   //roller(350, 100);
  //   stopBase();
    
  // }

  // void rightHalf() {
  //   resetTotalDistance();  
  //   //pto6();


  //   //forwardInchesTimed(1.5, 5, 750);
  //   setBase(5); // move into rollers
  //   task::sleep(300);
  //   holdBase();

  //   //roller(275,100);
  //   //setIntake(50);

  //           //wait(200, msec);
  //           //setIntake(0);
  //          //stopBase();
          
  //  // back up from roller and turn towards center of field
  //   //forwardPID(-2.8, 35);
  //   forwardPID(-2.8, 20);
  //   turnPID(45.6, 25);

  //   setIntake(100);
  //   //forwardPIDIncr(-30, 70); // go to center
  //   setIntake(0);
  //   task::sleep(50);
    
  //   //gyroTurn(-45, 20);
  //   //setIntake(100);
  //   //forwardPID(-30, 55); // go to center
  //   //turnPID(-44.15, 22, 1750); // turn to shoot
  //   turnPID(-43.75, 22, 1750); // turn to shoot
  //   forwardInches(-0.8, 13);
  //   setIntake(100);
  //   task::sleep(500);
  //   setIntake(0);
  //   task::sleep(100);
    
  //   R2BASE.spin(reverse, 100, pct); // shoot
  //   wait(600, msec);
  //   //task pog(catapultReset);

  //   task::sleep(1000);
  //   // pick up to shoot again
  //   setIntake(100);
  //   forwardInches(6, 17);
  //   forwardInches(-6, 17);
    
  //   gyroTurn(-75, 22); // turn to other disk
    
  //   setIntake(100);
  //   forwardInches(8, 17);
  //   forwardInches(-7, 17);
    
  //   //turnPID(-44, 22, 2500); // turn to shoot
  //   turnPID(-44, 22, 2500); // turn to shoot
  //   setIntake(0);
  //   forwardInches(-0.7, 12);

  //   R2BASE.spin(reverse, 100, pct); // shoot
  //   wait(600, msec);
  //   //task m(catapultReset);

  //   stopBase();


  // }

  // void setonSkills(){
  //   resetTotalDistance();  
  //   //pto6();

  //   //forwardInchesTimed(1.5, 5, 750);
  //   setBase(5); // move into rollers
  //   task::sleep(300);
  //   holdBase();

  //   //roller(450,100);
  //   //setIntake(50);

  //           //wait(200, msec);
  //           //setIntake(0);
  //          //stopBase();
          
  //  // back up from roller and turn towards center of field
  //   //forwardPID(-2.8, 35);
  //   forwardInches(-1.5, 20);
  //   turnPID(138, 25);
  //   setIntake(100);
  //   forwardInches(6, 20);
    
  //   gyroTurn(145, 20);
  //   forwardInches(6, 20);
  //   turnPID(90, 26);
  //   //setIntake(0);

  //  // move into rollers
  
  //   setIntake(0);
  //   forwardInchesTimed(2.25, 15, 1500);
  //   setBase(5);
  //   task::sleep(275);
  //   holdBase();

  //   //roller(475, 100);

  //   forwardInches(-1.5, 15);
  //   turnPID(-2.3, 27);
  //   setIntake(100);
  //   forwardPID(-26, 45);
    
  //   gyroTurn(0.5, 23);
  //   R2BASE.spin(reverse, 100, pct); // shootol
  //   wait(600, msec);
  //   //task m(catapultReset);
    
  //   wait(100, msec);

  //   forwardInches(4, 20);
    
  //   turnPID(-60, 23);
  //   forwardPID(17, 30);
  //   setIntake(100);
  //   turnPID(-147, 27);

  // }

// old autos
  // void leftFull(){
  //   // have to reset these manually before every auton
  //   resetTotalDistance();
  //   resettingSling = false; // stop resetting cata if manually pressed


  //   setBase(5); // move into rollers
  //   task::sleep(300);//450
  //   holdBase();

  //   //roller(275,100);
  //   //setIntake(50);

  //           //wait(200, msec);
  //           //setIntake(0);
  //          //stopBase();
            

  //  // back up from roller and turn towards center of field
  //   forwardPID(-2.8, 35);
  //   turnPID(-45.6, 25);

  //   setIntake(100);
  //  //forwardPIDIncr(-30, 70); // go to center
  //   setIntake(0);
  //   task::sleep(50);
    
  //   //gyroTurn(-45, 20);
  //   //setIntake(100);
  //   //forwardPID(-30, 55); // go to center
  //   turnPID(44.15, 22, 1750); // turn to shoot
  //   forwardInches(-0.9, 16);
  //   setIntake(100);
  //   task::sleep(500);
  //   setIntake(0);
  //   task::sleep(100);
    
  //   R2BASE.spin(reverse, 100, pct); // shoot
  //   wait(600, msec);
  //   //task pog(catapultReset);

  //   setIntake(100);
  //   task::sleep(750);
  //   setIntake(0);
  //   //task::sleep(100);
  //   /* // shoot twice
  //   R2BASE.spin(reverse, 100, pct); // shoot
  //   wait(800, msec);
  //   task pog(catapultReset);
  //   wait(250, msec);*/

  //   forwardInches(0.75, 22);

  //   //turnPID(-44, 28);
  //   gyroTurn(-46.5, 25);
  //   forwardInches(-28, 100);
    
  //   gyroTurn(91, 30);
  //   //turnPID(90, 30);

  //   forwardInchesTimed(4.25, 50, 2500); // move into rollers
  //   setBase(5); // move into rollers
  //   task::sleep(500);
  //   holdBase();

  //   //roller(350, 100);
  //   stopBase();
    
  // }

  // void rightHalf() {
  //   resetTotalDistance();  
  //   //pto6();


  //   //forwardInchesTimed(1.5, 5, 750);
  //   setBase(5); // move into rollers
  //   task::sleep(300);
  //   holdBase();

  //   //roller(275,100);
  //   //setIntake(50);

  //           //wait(200, msec);
  //           //setIntake(0);
  //          //stopBase();
          
  //  // back up from roller and turn towards center of field
  //   //forwardPID(-2.8, 35);
  //   forwardPID(-2.8, 20);
  //   turnPID(45.6, 25);

  //   setIntake(100);
  //   //forwardPIDIncr(-30, 70); // go to center
  //   setIntake(0);
  //   task::sleep(50);
    
  //   //gyroTurn(-45, 20);
  //   //setIntake(100);
  //   //forwardPID(-30, 55); // go to center
  //   //turnPID(-44.15, 22, 1750); // turn to shoot
  //   turnPID(-43.75, 22, 1750); // turn to shoot
  //   forwardInches(-0.8, 13);
  //   setIntake(100);
  //   task::sleep(500);
  //   setIntake(0);
  //   task::sleep(100);
    
  //   R2BASE.spin(reverse, 100, pct); // shoot
  //   wait(600, msec);
  //   //task pog(catapultReset);

  //   task::sleep(1000);
  //   // pick up to shoot again
  //   setIntake(100);
  //   forwardInches(6, 17);
  //   forwardInches(-6, 17);
    
  //   gyroTurn(-75, 22); // turn to other disk
    
  //   setIntake(100);
  //   forwardInches(8, 17);
  //   forwardInches(-7, 17);
    
  //   //turnPID(-44, 22, 2500); // turn to shoot
  //   turnPID(-44, 22, 2500); // turn to shoot
  //   setIntake(0);
  //   forwardInches(-0.7, 12);

  //   R2BASE.spin(reverse, 100, pct); // shoot
  //   wait(600, msec);
  //   //task m(catapultReset);

  //   stopBase();


  // }

  // void setonSkills(){
  //   resetTotalDistance();  
  //   //pto6();

  //   //forwardInchesTimed(1.5, 5, 750);
  //   setBase(5); // move into rollers
  //   task::sleep(300);
  //   holdBase();

  //   //roller(450,100);
  //   //setIntake(50);

  //           //wait(200, msec);
  //           //setIntake(0);
  //          //stopBase();
          
  //  // back up from roller and turn towards center of field
  //   //forwardPID(-2.8, 35);
  //   forwardInches(-1.5, 20);
  //   turnPID(138, 25);
  //   setIntake(100);
  //   forwardInches(6, 20);
    
  //   gyroTurn(145, 20);
  //   forwardInches(6, 20);
  //   turnPID(90, 26);
  //   //setIntake(0);

  //  // move into rollers
  
  //   setIntake(0);
  //   forwardInchesTimed(2.25, 15, 1500);
  //   setBase(5);
  //   task::sleep(275);
  //   holdBase();

  //   //roller(475, 100);

  //   forwardInches(-1.5, 15);
  //   turnPID(-2.3, 27);
  //   setIntake(100);
  //   forwardPID(-26, 45);
    
  //   gyroTurn(0.5, 23);
  //   R2BASE.spin(reverse, 100, pct); // shootol
  //   wait(600, msec);
  //   //task m(catapultReset);
    
  //   wait(100, msec);

  //   forwardInches(4, 20);
    
  //   turnPID(-60, 23);
  //   forwardPID(17, 30);
  //   setIntake(100);
  //   turnPID(-147, 27);

  // }









// void turnPID(float target, float maxPower, int msTimeout) {
  //   float change = 1.0;
    
  //   // new constants for Kalahari
  //   float Kp = 0.48;       // 0.508497;
  //   float Ki = 0.005;//75;//19;      // 11; //0.007;
  //   float Kd = 0.499;//3; // 0.0504;//051;//.09;

  //   float error = (target * change) - getRotationDeg();
  //   float lastError;
  //   float integral;
  //   float derivative;

  //   float integralPowerLimit =
  //       40 / Ki;                   // little less than half power in pct (percent)
  //   float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks

  //   float exitThreshold = 0.75; // Exit loop when error is less than this
  //   float finalPower;

  //   resetTotalDistance();
  //   Brain.resetTimer();

  //   while (fabs(error) > exitThreshold &&
  //          Brain.timer(vex::timeUnits::msec) < msTimeout) {
  //     Brain.Screen.printAt(140, 95, "ROTATION: %.3f deg", getRotationDeg());
  //     error = (target * change) - getRotationDeg();

  //     if (fabs(error) < integralActiveZone && error != 0) {
  //       integral = integral + error;
  //     } else {
  //       integral = 0;
  //     }
  //     integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

  //     derivative = error - lastError;
  //     lastError = error;

  //     finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
  //     finalPower = keepInRange(finalPower, -maxPower, maxPower);

  //     setLeftBase(-finalPower);
  //     setRightBase(finalPower);

  //     // Brain.Screen.printAt(140, 25,"P: %.2f, I: %.2f, D: %.2f", (Kp * error),
  //     // (Ki * integral), (Kd * derivative));
  //     Brain.Screen.printAt(140, 65, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
  //                          (Ki * integral), (Kd * derivative));
  //     Brain.Screen.printAt(140, 25, "INERTIAL2: %.2f", getRotationDeg());

  //     vex::task::sleep(20);
  //   }
  //   stopBase();
  // }








// double initialAngle = 0;
// double sidewaysError = 0;

// void resetDriveStraight() {
//   initialAngle = getRotationDeg();
//   sidewaysError = 0;
//   correctionL = 0;
//   correctionR = 0;
// }


// // Try to go back to original straight line by tracking shifting sideways.
// int driveStraight() {
//   double Kp = 0.4; // multiply error by this to get speed

//   while (true) {  
//     double angleError = initialAngle - getRotationDeg();
//     double deltaSideways = /*fwd_derivative*/1 * sin(angleError * toRadians); // get change in sideways distance
//     sidewaysError += deltaSideways; // accumulate sideways distance

//     // get correction speed
//     double cSpeed = angleError * Kp;
//     cSpeed = keepInRange(cSpeed, -5, 5);

//     correctionL = -cSpeed;
//     correctionR = cSpeed;
    
//     //Brain.Screen.printAt(170, 150, "ae:%.1f ds:%.1f, se:%.1f  ", angleError, deltaSideways, sidewaysError);
//     //Brain.Screen.printAt(170, 170, "cs:%.1f  ", cSpeed);
//     wait(10, msec);

//   }
// }



// void rollerLeftSketchy(){
//   // manually reset when testing w/ comp switch
//   resetTotalDistance();
//   resettingSling = false; // stop resetting cata if manually pressed

//   setBase(-5);
//   task::sleep(250);
//   XBOW.rotateTo(-600, deg, 100, velocityUnits::pct);
//   task::sleep(400);
  
//   forwardPID(6, 40);
  
//   //XBOW.rotateTo(0, deg, 100, velocityUnits::pct);
//   // forwardInchesTimed(-3.5, 15, 750);
//   // forwardInchesTimed(2, 30, 2000);
// }


// void leftHalf(){
//   forwardInchesTimed(-3.5, 28, 350);
//   forwardInchesTimed(2.25, 30, 500);
  
//   forwardInchesTimed(-3.5, 28, 350);
//   forwardInchesTimed(2.25, 30, 500);

//   forwardInchesTimed(-3.5, 28, 350);
//   forwardInchesTimed(2.25, 30, 500);

//   forwardInchesTimed(-3.5, 28, 350);
//   forwardInchesTimed(2.25, 30, 500);

//   forwardInches(1.5, 20);
//   forwardInches(1, 10);

//   turnPID(-45.5, 25);
//   //forwardPID(70,50);
//   task::sleep(50);
//   forwardPIDGradual(70,57);
//   turnPID(45,23); // turn to shoot
//   task::sleep(50);

//   lockOff();
//   task::sleep(250);
// }

// void leftFull(){
  
//   forwardInchesTimed(-3.5, 28, 350);
//   forwardInchesTimed(2.25, 30, 500);
  
//   forwardInchesTimed(-3.5, 28, 350);
//   forwardInchesTimed(2.25, 30, 500);

//   forwardInchesTimed(-3.5, 28, 350);
//   forwardInchesTimed(2.25, 30, 500);

//   forwardInchesTimed(-3.5, 28, 350);
//   forwardInchesTimed(2.25, 30, 500);

//   forwardInches(1.5, 20);
//   forwardInches(1, 10);

//   turnPID(-46, 25);
//   //forwardPID(70,50);
//   task::sleep(50);
//   forwardPIDGradual(70,57);
//   turnPID(45,23); // turn to shoot
//   task::sleep(50);
//   /*task v(shootSlingAsync);
//   task::sleep(1800);*/
  
//   lockOff();
//   task::sleep(250);
//   turnPIDDist(-8, 22); // inertial sensor angle doesnt work for some reason so have to turn by distance
//   //turnPID(-44.3, 24);

//   task::sleep(100);
//   forwardPIDGradual(68.5,57);
  
//   //task::sleep(100);
//   //turnPID(90, 25);
//   turnPIDDist(12.25, 22); // inertial sensor angle doesnt work for some reason so have to turn by distance
//   //turnPIDDist(-8, 22);

//   forwardInchesTimed(-10, 32, 450);
  
//   forwardInchesTimed(2.5, 30, 1000);

//   forwardInchesTimed(-3.5, 32, 750);
//   forwardInchesTimed(2.5, 30, 1000);
  
//   forwardInchesTimed(-3.5, 32, 750);
//   forwardInchesTimed(2.5, 30, 1000);

//   forwardInchesTimed(-3.5, 32, 750);
//   forwardInchesTimed(2.5, 30, 1000);

//   //forwardInchesTimed(-3.5, 15, 750);
//   //forwardInchesTimed(2, 30, 2000);
  
//   //forwardInchesTimed(-3.5, 15, 750);
//   //forwardInchesTimed(2, 30, 2000);


// }



// void leftRoller() {
//   forwardInchesTimed(-3,45, 750);
//   forwardInchesTimed(2, 30, 1000);
  
//   task::sleep(500);
//   forwardInchesTimed(-3, 45, 750);
//   forwardInchesTimed(3, 30, 1000);
//   holdBase();
//   task::sleep(500);
  
//   forwardInchesTimed(-6, 45, 750);
//   forwardInchesTimed(3, 25, 1000);
//   task::sleep(500);
//   holdBase();
  
//   forwardInchesTimed(-6, 45, 750);
//   forwardInchesTimed(3, 25, 1000);
//   task::sleep(500);
//   holdBase();

//   forwardInchesTimed(-6, 45, 750);
//   forwardInchesTimed(3, 25, 1000);
// }

// void rollerRight() {
//   forwardPID(18, 30);
//   turnPID(-90, 25);
//   forwardInchesTimed(7, 30, 2000);

// }

// void rightHalf() {
  
//   forwardPID(-25, 30);
//   gyroTurn(-88.5, 24);
//   forwardInchesTimed(-13, 32, 750);
  
//   forwardInchesTimed(2.5, 30, 1000);

//   forwardInchesTimed(-3.5, 32, 750);
//   forwardInchesTimed(2.5, 30, 1000);
  
  
//   forwardInchesTimed(-3.5, 32, 750);
//   forwardInchesTimed(2.5, 30, 1000);

//   forwardInchesTimed(-3.5, 32, 750);
//   forwardInchesTimed(2.5, 30, 1000);

//   /*
//   forwardInchesTimed(-3.5, 15, 750);
//   forwardInchesTimed(3, 20, 3000);
//   //task b(shootSling());
//   turnPID(45, 25); // turn to midfield
//   task::sleep(400);
//   forwardPID(32.5, 40);
//   turnPID(-45, 25); // turn to goal
//   task::sleep(400);
//   shootSling();*/

// }

// void rightDiscs(){
//   forwardInches(10, 12);

//   turnPID(44.5, 25);
//   //forwardPID(70,50);
//   task::sleep(50);
//   forwardPIDGradual(52,57);
//   turnPID(-45,23); // turn to shoot
//   task::sleep(250);

//   lockOff();
//   task::sleep(500);
// }



/*
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
}*/



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




// void right2fsdkpf() {
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
  
//   /*task banana(resetSlingAsync);
//   turnPIDFast(-110, 27);

//   forwardInches(-11.5, 24);
  
//   turnPID(-45, 25);
//   task::sleep(100);
  
//   task pineapple(delayIntake);
//   forwardPIDGradual(67.5, 50);
//   turnPID(-135, 25);
//   stopIntake();
//   lockOff();*/
// }



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


// // # 2 kali hotel with backup
// void right2() {
//   // ROLLER #1 ////////////////
//   setBase(-10);
//   task::sleep(350);
//   setBase(-2);

//   //stopBase();
//   INTAKE.setTimeout(5000, msec);
//   roller(360, 100);
//   //lockOff();
//   //task s(resetSlingAsync);
//   forwardInches(7, 19);
//   turnPIDFast(-6, 24);
//   task::sleep(500);
//   lockOff();
// }


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




// void forwardPID(double targetInches, double maxPower, int msTimeout) {
//   float change = 1;
//   float Kp = 0.3031; // you need to tune these value manually. 0.305
//   float Ki = 0.008;// //.03; // they are just arbitrary constants so you need to
//                      //test     0.0145
//   float Kd = 0.53231; //.18922175;    //until they work. 0.529

//   float error = inchesToTicks(targetInches) - getBaseAvg(); // desired - actual
//   float lastError;
//   float integral;
//   float derivative;

//   float integralPowerLimit =
//       40 / Ki; // little less than half power in pct (percent)
//   float integralActiveZone = 15;
//   // explaining integral active zone
//   // area where proportion is effective is represented with >>/<<
//   // area where proportion is not strong enough to move base rep with ----
//   // 0 is the target where error is equal to 0
//   // only want integral during ineffective zone to prevent windup of power
//   //>>>>>>>>>>>>-----------0---------------<<<<<<<<<<<<<<<<

//   float exitThreshold = 0.5; // Exit loop when error is less than this
//   float finalPower;

//   resetTotalDistance();
//   Brain.resetTimer();

//   while (fabs(error) > exitThreshold &&
//          (Brain.timer(vex::timeUnits::msec) < msTimeout || msTimeout == -1)) {
//     // Brain.Screen.printAt(140, 85,"ROTATION: %.3f deg", getRotation());
//     error = inchesToTicks(targetInches) - getBaseAvg();

//     if (fabs(error) < integralActiveZone && error != 0) {
//       integral = integral + error;
//     } else {
//       integral = 0;
//     }
//     integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

//     derivative = error - lastError;
//     lastError = error;
//     /*
//      if (error == 0)
//      {
//        derivative = 0;
//      }*/

//     finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
//     finalPower = keepInRange(finalPower, -maxPower, maxPower);

//     setLeftBase(finalPower);
//     setRightBase(finalPower);
//     Brain.Screen.printAt(210, 150, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
//                          (Ki * integral), (Kd * derivative));
//     Brain.Screen.printAt(
//         210, 170, "Dist: %.2f, Error: %.2f", ticksToInches(getBaseAvg()), ticksToInches(error));
//     vex::task::sleep(40);
//   }
//   stopBase();
// }


// void turnPID(double target, double maxPower, int msTimeout) {
//   float change = 1.0;
//   /*
//     float Kp = 0.5105;   //getting to target
//     float Ki = 0.00825; // increases speed (builds up over time) before: 0.008
//     float Kd = 0.0111;*/    //slow down

//   /*float Kp = 0.6;    // 0.508497;
//   float Ki = 0.0001; // 0.007;
//   float Kd = 0.0504; // 051;//.09;*/

//   // new constants for Kalahari
//   float Kp = 0.48;       // 0.508497;
//   float Ki = 0.005;//75;//19;      // 11; //0.007;
//   float Kd = 0.499;//3; // 0.0504;//051;//.09;

//   float error = (target * change) - getRotationDeg();
//   float lastError;
//   float integral;
//   float derivative;

//   float integralPowerLimit =
//       40 / Ki;                   // little less than half power in pct (percent)
//   float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks
//   // explaining integral active zone
//   // area where proportion is effective is represented with >>/<<
//   // area where proportion is not strong enough to move base rep with ----
//   // 0 is the target where error is equal to 0
//   // only want integral during ineffective zone to prevent windup of power
//   //>>>>>>>>>>>>-----------0---------------<<<<<<<<<<<<<<<<

//   float exitThreshold = 0.75; // Exit loop when error is less than this
//   float finalPower;

//   resetTotalDistance();
//   Brain.resetTimer();

//   while (fabs(error) > exitThreshold &&
//          (Brain.timer(vex::timeUnits::msec) < msTimeout || msTimeout == -1)) {
//     Brain.Screen.printAt(140, 95, "ROTATION: %.3f deg", getRotationDeg());
//     error = (target * change) - getRotationDeg();

//     if (fabs(error) < integralActiveZone && error != 0) {
//       integral = integral + error;
//     } else {
//       integral = 0;
//     }
//     integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

//     derivative = error - lastError;
//     lastError = error;
//     /*
//      if (error == 0)
//      {
//        derivative = 0;
//      }*/

//     finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
//     finalPower = keepInRange(finalPower, -maxPower, maxPower);

//     setLeftBase(-finalPower);
//     setRightBase(finalPower);
//     // Brain.Screenst.printAt(140, 25,"P: %.2f, I: %.2f, D: %.2f", (Kp * error),
//     // (Ki * integral), (Kd * derivative));
//     Brain.Screen.printAt(140, 65, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
//                          (Ki * integral), (Kd * derivative));
//     Brain.Screen.printAt(140, 45, "INERTIAL: %.2f", INERTIAL.value());
//     Brain.Screen.printAt(140, 25, "INERTIAL2: %.2f", getRotationDeg());

//     vex::task::sleep(40);
//   }
//   stopBase();
// }


// void turnPIDDist(double targetInches, double maxSpeed, int msTimeout) {
//   // TUNE THESE
//   double Kp = 9; 
//   double Ki = 0.00;
//   double Kd = 4;

//   double errorThreshold = 0.125; // Only exit loop when error is less than this
//   double derTheshold = 0.005; // Only exit loop when derivative is less than this
//   double minSpeed = 1; // stay at least this fast to overcome friction. May be more reliable than using integral
//   double integralPowerLimit = 5 / Ki; // how much speed the integral can contribute
//   double integralActiveZone = 5; // How close to error the integral adds up
  
//   resetTotalDistance();
//   //resetDriveStraight();
//   Brain.resetTimer();
//   double integral;
//   double lastError;
//   bool continueLoop = true;

//   while (continueLoop) {
//     // ERROR: DISTANCE FROM TARGET. MOST SPEED COMES FROM HERE
//     double error = targetInches - ticksToInches(R1BASE.rotation(deg)+R2BASE.rotation(deg)+R3BASE.rotation(deg))/3; // desired - actual

//     // INTEGRAL: ACUMULATE SPEED TO GET FASTER AT END
//     integral = integral + error;
//     if (fabs(error) > integralActiveZone) { integral = 0; } // reset integral if outside range
//     integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit); // keep integral within range

//     // DERIVATIVE: SLOW DOWN TOWARDS END
//     double derivative = error - lastError;
//     lastError = error;

//     double speed = ((Kp * error) + (Ki * integral) + (Kd * derivative));
//     speed = keepInRange(speed, -maxSpeed, maxSpeed);
//     if (speed > 0 && speed < minSpeed)  { // go at least minspeed forward
//       speed = minSpeed;
//     } else if (speed > -minSpeed && speed < 0) { // go at least minspeed backward
//       speed = -minSpeed;
//     }
//     setLeftBase(-speed/* + correctionL*/);
//     setRightBase(speed/* + correctionR*/);
//     Brain.Screen.printAt(10, 180, "P: %.2f, I: %.2f, D: %.2f", (Kp * error), (Ki * integral), (Kd * derivative));
//     Brain.Screen.printAt(10, 200, "Dist: %.2f, Error: %.2f", getTotalDistance(), error);
//     vex::task::sleep(10);
//     //graph(error);
//     // Exit loop if within certain distance AND slow enough to not overshoot
//     if (fabs(error) < errorThreshold && fabs(derivative) <= derTheshold) { continueLoop = false; }
//     // Exit loop if run out of time. No timeout by default if not specified (set to -1).
//     if (Brain.timer(vex::timeUnits::msec) > msTimeout && msTimeout != -1) { continueLoop = false; }
//   }
//   stopBase();
// }


// SKILLS //
// void skills() {
//   // LB ROLLER ////////////////
//   setBase(-10);
//   task::sleep(350);
//   setBase(-2);
//   roller(360, 100);
//   //wait(0.15, sec);
//   task jsdhgkj(intakeDelay);
//   forwardInches(26, 22);
//   task::sleep(500);

//   // RR ROLLER ///////////////
//   turnPID(-90, 37);
//   task gsohgo(stopIntakeDelay);
//   forwardInchesTimed(-25, 45, 1000);
//   setBase(-2);
//   roller(360, 100);
//   stopBase();

//   // B GOAL #1 ///////////////
//   forwardInches(10, 30);
//   turnPID(-3, 26);
//   forwardPIDGradual(45, 45);
//   lockOff();
//   task::sleep(500);

//   // INTAKE RED LINE OF 3 /////////////////
//   task rbvhsvhdv(resetSlingAsync);
//   forwardPID(-36, 30);
//   task::sleep(3000);
//   turnPID(-45,30);
//   setIntake(100);
//   forwardPIDGradual(64, 40);
//   task::sleep(400);
//   forwardInches(-3.5, 10);

//   // BLUE GOAL #2 //////////// 
//   turnPID(45, 26);
//   stopIntake();
//   forwardInches(9, 12);
//   task::sleep(100);
//   lockOff();
//   task::sleep(100);
//   forwardInches(-5, 12);
//   turnPID(-55, 30);
//   task::sleep(100);

//   // LR ROLLER ///////////////
//   forwardPIDGradual(56, 55);
//   turnPID(-180, 25);
//   forwardInchesTimed(-20, 25, 3500);
//   roller(360, 100);
//   forwardPID(25, 30);

//   // RB ROLLER ///////////////
//   turnPID(-270, 25);
//   forwardInchesTimed(-32, 30, 6000);
//   roller(360, 100);

//   // ENDGAME ////////////////
//   forwardInches(29, 26);
//   turnPID(-270+46, 23);
//   forwardInches(-30, 26);
//   manspread(); // usually hits 20-24 tiles

//   // tiles: 21
//   // RockRidge - #1: 104
// }

// // SKILLS FOR STIFF ROLLERS //
// void skillsStiff() {
//   // LB ROLLER ////////////////
//   setBase(-10);
//   task::sleep(350);
//   setBase(-2);
//   roller(500, 100);
//   //wait(0.15, sec);
//   task jsdhgkj(intakeDelay);
//   forwardInches(26, 22);
//   task::sleep(500);

//   // RR ROLLER ///////////////
//   turnPID(-90, 37);
//   task gsohgo(stopIntakeDelay);
//   forwardInchesTimed(-25, 45, 1000);
//   setBase(-2);
//   roller(520, 100);
//   stopBase();

//   // B GOAL #1 ///////////////
//   forwardInches(10, 30);
//   turnPID(-3, 26);
//   forwardPIDGradual(45, 45);
//   lockOff();
//   task::sleep(500);

//   // INTAKE RED LINE OF 3 /////////////////
//   task rbvhsvhdv(resetSlingAsync);
//   forwardPID(-36, 30);
//   task::sleep(3000);
//   turnPID(-45,30);
//   setIntake(100);
//   forwardPIDGradual(64, 40);
//   task::sleep(400);
//   forwardInches(-3.5, 10);

//   // BLUE GOAL #2 //////////// 
//   turnPID(45, 26);
//   stopIntake();
//   forwardInches(9, 12);
//   task::sleep(100);
//   lockOff();
//   task::sleep(100);
//   forwardInches(-5, 12);
//   turnPID(-55, 30);
//   task::sleep(100);

//   // LR ROLLER ///////////////
//   forwardPIDGradual(56, 55);
//   turnPID(-180, 25);
//   forwardInchesTimed(-20, 25, 3500);
//   roller(500, 100);
//   forwardPID(25, 30);

//   // RB ROLLER ///////////////
//   turnPID(-270, 25);
//   forwardInchesTimed(-32, 30, 6000);
//   roller(500, 100);

//   // ENDGAME ////////////////
//   forwardInches(29, 26);
//   turnPID(-270+46, 23);
//   forwardInches(-30, 26);
//   manspread(); // usually hits 20-24 tiles
// }




// // #LEFT WP SHOOT FROM ROLLER //
// void leftFullOld() {
//   // ROLLER #1 ////////////////
  
//   setBase(-10);
//   task::sleep(350);
//   setBase(-2);

//   roller(300, 100);
//   forwardInches(13, 19);
//   turnPID(8, 10);
//   task::sleep(250);
//   lockOff();
//   task::sleep(300);
//   turnPID(-45, 22);
//   //gyroTurn(-43, 15);
//   // //turnPID(-44, 23);
  
//   task::sleep(500);
//   forwardPIDGradual(123, 65); //125 in


//   //turnL(240, 27);
//   turnL(240, 22);
//   //gyroTurn(135, 30);//turnPID(135, 24);

//   setIntake(100);
//   //revConst(150, 15, 3000);
  
//   revConst(120, 15, 4000);
//   //revConst(30, 6, 5000);
//   stopIntake();

//   controllerPrim.rumble(".");
//   //forwardInches(1, 1)
  
//   //revConst(200, 20, 1500);

//   //setBase(-2);
//   //roller(375, 100);
//   //roller(425, 100);
//   //stopBase();
// }


// // LEFT WP SHOOT FROM MIDFIELD
// void leftFullMid() {
//   // ROLLER #1 ////////////////
//   setBase(-10);
//   task::sleep(350);
//   setBase(-2);

//   //stopBase();

//   roller(300, 100);
//   //lockOff();
//   //task s(resetSlingAsync);
//   forwardInches(5, 22);
//   //gyroTurnSlow(-38, 20);
//   //gyroTurnSlow(-47, 25);
//   turnPID(-50, 21);

//   forwardPIDGradual(69, 55);
//   // SHOOT AT HIGH GOAL //////////
//   //gyroTurnSlow(43, 32);
//   turnPID(42.5, 25);
//   task::sleep(100);
//   forwardInches(-2, 13);
//   task::sleep(200);
//   toggleLock();
//   task::sleep(200);
//   forwardInches(2, 13);

//   // TURN TO ROLLER #2  /////////
//   //turnPID(-35, 32);
//   //turnR(155, 18);
//   turnPID(-34.5, 25);
//    forwardPIDGradual(68, 60); // 65 in
//   revConst(50, 22);

//    turnL(210, 27);//220
//   revConst(200, 33, 550);
//   stopBase();
//   roller(340, 100);
// }



// void leftFull()
// {

//             //task k(resetSlingAsync);
//             //curvePID(9, 25, 2.375);
//             //curvePID(14, 40, 2);
//             //task::sleep(99999);

//             /*etBase(-10);
//             task::sleep(300);
//             //setBase(-2);*/
//             //task::sleep(250);



//   // forwardInches(-1.5, 10);
//   // holdBase();

//   // roller(300, 100);
//   // task::sleep(50);
//   // setIntake(100);
//   // forwardPID(6, 16);
//   // turnPIDSmall(4, 12);

//   // stopIntake();
//   // task::sleep(100);
//   // lockOff();
//   // task::sleep(75);

//   // task b(resetSlingAsync);
//   // turnPID(-58.5,20); // trun to 3 stack MESSES UP GOES TOO RIGHT SOMETIMES
//   // task::sleep(100);
//   // munchOn();

//   // task c(intakeDelayMunch);
//   // forwardPID(45,33); // to 3 stack
//   // task::sleep(50);
//   // turnPID(26.5, 32);
//   // forwardPID(-7.5, 28);
//   // stopIntake();
//   // task::sleep(75);
//   // lockOff(); // shot #2
  
//   // task d(resetSlingAsync);
//   // task::sleep(75);
//   // //curvePID(9, 26, 2.375);
//   // curvePID(17.45, 25, 2.375);
//   // task::sleep(50);
//   // task e(intakeDelaySafe);

//   // forwardPID(85, 75);
//   // turnL(250, 30);
//   // forwardInches(-5, 50);






//           //munchOff();

//           /*
//             task a(shootWhileMove);
//             forwardPID(12, 100);
//             holdBase();
//             forwardInches(-15, 30);
//             roller(360, 100);

//             task b(resetSlingAsync);
//             munchOn();
//             forwardPID(5, 20);
//             turnPID(-55, 20);
//             waitUntil(!resettingSling);
//             setIntake(100);
//             task c(downMunch);
//             forwardPID(60, 30);
//             munchOff();*/



// }



// double lastGraphX = 0;
// double lastGraphY = 0;
// // Display the graph of a variable over time
// void graph(double value, color graphColor=color::black) {
//   double x = timer::system()/30;
//   double y = (value) * 8;
//   // scale height so it is visible but also fits on the screen


//   // draw y = 0 line
//   Brain.Screen.setPenWidth(2);
//   Brain.Screen.setPenColor(color(255, 255, 255));
//   Brain.Screen.drawLine(0, 136, 480, 136);

//   // Draw data
//   Brain.Screen.setPenWidth(0);
//   Brain.Screen.setFillColor(graphColor);
//   //Brain.Screen.drawLine(lastGraphX, -lastGraphY + 136, x, -y + 136);
//   Brain.Screen.drawCircle(x, -lastGraphY + 136, 1);
  
//   Brain.Screen.setPenColor(color(255, 255, 255));
//   lastGraphX = x;
//   lastGraphY = y;
// }




  // void lerpTester(double inches, int startSpeed, int endSpeed)
  // {
  //   resetTotalDistance();
  //   double targetDistance = inches;
  //   //double error = targetDistance;

  //   while(true)
  //   {
  //     double t = getTotalDistance()/targetDistance; // ratio of distance travelled (starts at 0 and ends at 1)
  //     double speed = startSpeed * t + endSpeed * (1 - t); // Gradually transition between startSpeed and endSpeed

  //     if( t >= 1){
  //       break;
  //     }

  //     setLeftBase(speed);
  //     setRightBase(speed);
  //   }
  //   stopBase();
  // }

  
// void leftFull()
// {
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



//   forwardInchesTimed(-0.75, 5, 1000);
//   holdBase();

//   roller(300, 100);
//   task::sleep(50);

//   //forwardPID(4,20);
  
//   curvePID(12, 20, 3.5);
//   //turnPID(-53,20);
//   setIntake(100);//80
//   //munchOn();
  
//   //task esg(LFintakeDelay);
//   // /forwardPID(45, 45);//45
  
//   //setIntake(100);
  
//   //task fsfl(munchOffDelay);
//   forwardPIDStraight(49, 37, -55);

//   //turnPID(33, 34);
//   gyroTurn(33, 35);
//   stopIntake();
//   forwardPIDStraight(-10, 18, 34);
  
//   task::sleep(75);
//   lockOff();
//   task::sleep(50);
  
//   task fsldfks(resetSlingAsync);
  
//   task::sleep(1000);
  
//   curvePID(16, 13, 2.5);

//   //turnPID(-44, 28);
//   //task::sleep(300);
//   //setIntake(80);

//   task fsldj(intakeDelaySafe80);
  
//   forwardPIDStraight(49,39, -45);
//   //forwardPID(50,40);

//   turnPID(67, 27);
//   lockOff();
//   gyroTurn(-45,34);
//   forwardPIDStraight(25,45, -45);

//  /*   //forwardPID(-4,18);
//     //task::sleep(100);
//   task::sleep(75);
//   lockOff();
  
// */
//   //curvePID(17.45, 25, 2.375);


//       // turnPIDSmall(6,12);
//       // lockOff();

//       // task a(shootWhileMove);
//       // forwardPID(12, 100);
//       // holdBase();

//   //turnPID(-5,12);

//   // setIntake(100);
//   // forwardPID(6, 16);
//   // turnPIDSmall(4, 12);

//   // stopIntake();
//   // task::sleep(100);
//   // lockOff();
//   // task::sleep(75);

//   // task b(resetSlingAsync);
//   // turnPID(-58.5,20); // trun to 3 stack MESSES UP GOES TOO RIGHT SOMETIMES
//   // task::sleep(100);
//   // munchOn();

//   // task c(intakeDelayMunch);
//   // forwardPID(45,33); // to 3 stack
//   // task::sleep(50);
//   // turnPID(26.5, 32);
//   // forwardPID(-7.5, 28);
//   // stopIntake();
//   // task::sleep(75);
//   // lockOff(); // shot #2
  
//   // task d(resetSlingAsync);
//   // task::sleep(75);
//   // //curvePID(9, 26, 2.375);
//   // curvePID(17.45, 25, 2.375);
//   // task::sleep(50);
//   // task e(intakeDelaySafe);

//   // forwardPID(85, 75);
//   // turnL(250, 30);
//   // forwardInches(-5, 50);

//           //munchOff();

//           /*
//             task a(shootWhileMove);
//             forwardPID(12, 100);
//             holdBase();
//             forwardInches(-15, 30);
//             roller(360, 100);

//             task b(resetSlingAsync);
//             munchOn();
//             forwardPID(5, 20);
//             turnPID(-55, 20);
//             waitUntil(!resettingSling);
//             setIntake(100);
//             task c(downMunch);
//             forwardPID(60, 30);
//             munchOff();*/

// }

// void leftFull()
// {
//             //task k(resetSlingAsync);
//             //curvePID(9, 25, 2.375);
//             //curvePID(14, 40, 2);
//             //task::sleep(99999);

//             /*etBase(-10);
//             task::sleep(300);
//             //setBase(-2);*/
//             //task::sleep(250);

// // 2-25
// /*
//   setIntake(100);
//   forwardPID(4.5, 10);
  
//   turnPID(-40, 25);
//   task::sleep(150);
//   stopIntake();
//   lockOff();
//   task::sleep(100);
//   forwardInchesTimed(-10, 35, 2000);
//   roller(360, 100);
//   forwardPID(5, 15);
//   turnPID(-91, 26);
//   */



//   forwardInchesTimed(-0.75, 5, 1000);
//   holdBase();

//   roller(300, 100);
//   task::sleep(50);

//   //forwardPID(4,20);
  
//   curvePID(12, 20, 2.5);
//   //turnPID(-53,20);
//   setIntake(80);
//   //munchOn();
  
//   //task esg(LFintakeDelay);
//   // /forwardPID(45, 45);//45
  
//   //setIntake(100);
//   task fsfl(munchOffDelay);
//   forwardPIDStraight(48, 45, -55);

//   turnPID(36, 30);
//   stopIntake();
  
//   forwardPID(-14, 18);
  
//   task::sleep(75);
//   lockOff();
//   task::sleep(75);
  
//   task fsldfks(resetSlingAsync);
  
  
//   forwardPID(14,17);
//   turnPID(-44, 28);
  
//   task fsldj(intakeDelaySafe80);
//   forwardPID(50,40);
  

//   turnPID(67, 27);
//   /*forwardPID(-4,18);
//   task::sleep(100);*/
//   task::sleep(75);
//   lockOff();

//   //curvePID(17.45, 25, 2.375);


//       // turnPIDSmall(6,12);
//       // lockOff();

//       // task a(shootWhileMove);
//       // forwardPID(12, 100);
//       // holdBase();

//   //turnPID(-5,12);

//   // setIntake(100);
//   // forwardPID(6, 16);
//   // turnPIDSmall(4, 12);

//   // stopIntake();
//   // task::sleep(100);
//   // lockOff();
//   // task::sleep(75);

//   // task b(resetSlingAsync);
//   // turnPID(-58.5,20); // trun to 3 stack MESSES UP GOES TOO RIGHT SOMETIMES
//   // task::sleep(100);
//   // munchOn();

//   // task c(intakeDelayMunch);
//   // forwardPID(45,33); // to 3 stack
//   // task::sleep(50);
//   // turnPID(26.5, 32);
//   // forwardPID(-7.5, 28);
//   // stopIntake();
//   // task::sleep(75);
//   // lockOff(); // shot #2
  
//   // task d(resetSlingAsync);
//   // task::sleep(75);
//   // //curvePID(9, 26, 2.375);
//   // curvePID(17.45, 25, 2.375);
//   // task::sleep(50);
//   // task e(intakeDelaySafe);

//   // forwardPID(85, 75);
//   // turnL(250, 30);
//   // forwardInches(-5, 50);

//           //munchOff();

//           /*
//             task a(shootWhileMove);
//             forwardPID(12, 100);
//             holdBase();
//             forwardInches(-15, 30);
//             roller(360, 100);
//             task b(resetSlingAsync);
//             munchOn();
//             forwardPID(5, 20);
//             turnPID(-55, 20);
//             waitUntil(!resettingSling);
//             setIntake(100);
//             task c(downMunch);
//             forwardPID(60, 30);
//             munchOff();*/

// }
        