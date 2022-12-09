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
