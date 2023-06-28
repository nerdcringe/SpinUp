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

