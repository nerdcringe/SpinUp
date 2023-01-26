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
