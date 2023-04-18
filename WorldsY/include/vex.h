/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
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

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)


  
    /*
      LUPPER.spin(forward, Lspeed, velocityUnits::pct);
      RUPPER.spin(forward, Rspeed, velocityUnits::pct);
*/
    /*
      // If PTO is in 6 motor mode, use PTO motors like intake and rollers
      if (controllerPrim.ButtonR1.pressing()) {
        LUPPER.spin(fwd, 100, pct);
        RUPPER.spin(fwd, 100, pct);
      } else if  (controllerPrim.ButtonR2.pressing()) {
        LUPPER.spin(reverse, 100, pct);
        RUPPER.spin(reverse, 100, pct);
      } else {
        LUPPER.stop();
        RUPPER.stop();
      }*/


/*
// make highlight color either green or red depending if a variable is trrue or false
void setFillFor(bool var) {
  if (var) {
    Brain.Screen.setFillColor(color(10, 80, 40)); //green 
  } else {
    Brain.Screen.setFillColor(red);
  }
};
*/


/*

void rightHalf() {
  
      //setIntake(100);
  
          //forwardInches(15, 20);
  //     curveL(3.5, 17, 20);
  //     fwdConst(5, 30);
  //       task::sleep(99999);
  //       curveR(-5, 3, -75);

  //       curveR(10, 20, 19);
  //       task::sleep(99999);  
  turnPIDSmall(-16, 30, 600);
  //     controllerPrim.rumble("-");
  //     task::sleep(999999);
  lockOff();
  task::sleep(50);
  //task sdsjgj (pullbackAutoIntake);
  
  task::sleep(150);
          //setIntake(100);
  curveL(2.6, 9*1.6, 17.5*1.6);
            //task::sleep(99999);

            // turnPID(90, 30);
            // task::sleep(100);
            // forwardInches(1, 10);
        //turnPIDSmall(-16, 30, 2000);
        //task::sleep(250);
            //task::sleep(99999);
            //turnPID(-45, 25, 1500);
            //gyroTurn(45, 25);
        //curvePID(9.5, 35, 0.4);
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
  //curveL(1, 2*1.25, 20*1.25);
  //curveL(2, 19*1.25, 20*1.25);
  
  //curveR(-2, -5, -25);
  //curveL(1, 2, 25);
  //curveL(-2, -2, -20);
  //fwdConst(7, 30);


  //turnPID(getRotationDeg() + 180, 25);


  //forwardPID(-5, 25);
  //turnPID(-15, 25, 2000);
      //task bssfe(pullbackAuto);
}
*/