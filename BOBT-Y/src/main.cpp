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
  
  hello1 = 1;
  
  // fill screen with green bg
  Brain.Screen.setFillColor(color(10, 80, 30));
  Brain.Screen.setPenWidth(0);
  Brain.Screen.drawRectangle(0, 0, 480, 272);


  // Draw Y

  Brain.Screen.setPenColor(yellow);
  Brain.Screen.setPenWidth(40);
  Brain.Screen.drawLine(100, 125, 100, 220);
  Brain.Screen.drawLine(100, 130, 35, 35);
  Brain.Screen.drawLine(100, 130, 165, 35);

  R3BASE.resetPosition();
  L3BASE.resetPosition();
  
  //resetOdomPosition();
  resetTotalDistance();
  
  R3BASE.resetPosition();
  L3BASE.resetPosition();




  INERTIAL.calibrate();
  // Wait until the inertial sensor finishes calibrating
  while (INERTIAL.isCalibrating())
  {
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(fontType::mono40);
    Brain.Screen.printAt(100, 75, "INERTIAL SENSOR");
    Brain.Screen.printAt(100, 125, "CALIBRATING...");
    Brain.Screen.setFont(fontType::mono20);

  }

  Brain.Screen.setPenWidth(0);
  Brain.Screen.setFillColor(color(10, 80, 30));
  Brain.Screen.drawRectangle(0, 0, 480, 272); // make screen green

  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(210, 50, "Done calibrating");


  // Draw Y
  Brain.Screen.setPenColor(yellow);
  Brain.Screen.setPenWidth(40);
  Brain.Screen.drawLine(100, 125, 100, 220);
  Brain.Screen.drawLine(100, 130, 35, 35);
  Brain.Screen.drawLine(100, 130, 165, 35);


  resetTotalDistance();
}


// AUTON ///////////////////////////////////////////////////////////////////


void rollerAuton()
{
  // have to reset these manually before every auton
  resetTotalDistance();
  resettingCata = false; // stop resetting cata if manually pressed


  setBase(5); // move into rollers
  task::sleep(500);//450
  holdBase();

  roller(275,100);
 
 // back up from roller and turn towards center of field
  forwardPID(-1.5, 35);
}

void leftFull()
{
  // PID test
  //forwardPID(24, 35);

  
  //INERTIAL.setRotation(0, deg);
  
  //turnPID(89.875, 30);
  //turnPID(-45, 28);

  //forwardInchesTimed(1.5, 5, 750);
  //pto6();


  // have to reset these manually before every auton
  resetTotalDistance();
  resettingCata = false; // stop resetting cata if manually pressed


  setBase(5); // move into rollers
  task::sleep(300);//450
  holdBase();

  roller(275,100);
  //setIntake(50);

          //wait(200, msec);
          //setIntake(0);
         //stopBase();
          

 // back up from roller and turn towards center of field
  forwardPID(-2.8, 35);
  turnPID(-45.6, 25);

  setIntake(100);
 //forwardPIDIncr(-30, 70); // go to center
  setIntake(0);
  task::sleep(50);
  
  //gyroTurn(-45, 20);
  //setIntake(100);
  //forwardPID(-30, 55); // go to center
  turnPID(44.15, 22, 1750); // turn to shoot
  forwardInches(-0.9, 16);
  setIntake(100);
  task::sleep(500);
  setIntake(0);
  task::sleep(100);
  
  R2BASE.spin(reverse, 100, pct); // shoot
  wait(600, msec);
  task pog(catapultReset);

  setIntake(100);
  task::sleep(750);
  setIntake(0);
  //task::sleep(100);
  /* // shoot twice
  R2BASE.spin(reverse, 100, pct); // shoot
  wait(800, msec);
  task pog(catapultReset);
  wait(250, msec);*/

  forwardInches(0.75, 22);

  //turnPID(-44, 28);
  gyroTurn(-46.5, 25);
  forwardInches(-28, 100);
  
  gyroTurn(91, 30);
  //turnPID(90, 30);

  forwardInchesTimed(4.25, 50, 2500); // move into rollers
  setBase(5); // move into rollers
  task::sleep(500);
  holdBase();

  roller(350, 100);
  stopBase();
  
}

void rightFull(){
}

// half wp
void leftHalf(){
}

void rightHalfGood()
{
  resetTotalDistance();  
  //pto6();


  //forwardInchesTimed(1.5, 5, 750);
  setBase(5); // move into rollers
  task::sleep(300);
  holdBase();

  roller(275,100);
  //setIntake(50);

          //wait(200, msec);
          //setIntake(0);
         //stopBase();
        
 // back up from roller and turn towards center of field
  //forwardPID(-2.8, 35);
  forwardPID(-2.8, 20);
  turnPID(45.6, 25);

  setIntake(100);
  //forwardPIDIncr(-30, 70); // go to center
  setIntake(0);
  task::sleep(50);
  
  //gyroTurn(-45, 20);
  //setIntake(100);
  //forwardPID(-30, 55); // go to center
  //turnPID(-44.15, 22, 1750); // turn to shoot
  turnPID(-43.75, 22, 1750); // turn to shoot
  forwardInches(-0.8, 13);
  setIntake(100);
  task::sleep(500);
  setIntake(0);
  task::sleep(100);
  
  R2BASE.spin(reverse, 100, pct); // shoot
  wait(600, msec);
  task pog(catapultReset);

  task::sleep(1000);
  // pick up to shoot again
  setIntake(100);
  forwardInches(6, 17);
  forwardInches(-6, 17);
  
  gyroTurn(-75, 22); // turn to other disk
  
  setIntake(100);
  forwardInches(8, 17);
  forwardInches(-7, 17);
  
  //turnPID(-44, 22, 2500); // turn to shoot
  turnPID(-44, 22, 2500); // turn to shoot
  setIntake(0);
  forwardInches(-0.7, 12);

  R2BASE.spin(reverse, 100, pct); // shoot
  wait(600, msec);
  task m(catapultReset);

  stopBase();


}

void rightHalf()
{
  
  resetTotalDistance();  
  //pto6();


  //forwardInchesTimed(1.5, 5, 750);
  setBase(5); // move into rollers
  task::sleep(300);
  holdBase();

  roller(275,100);
  //setIntake(50);

          //wait(200, msec);
          //setIntake(0);
         //stopBase();
        
 // back up from roller and turn towards center of field
  //forwardPID(-2.8, 35);
  forwardPID(-2.8, 20);
  turnPID(45.6, 25);

  setIntake(100);
 // forwardPIDIncr(-30, 70); // go to center
  setIntake(0);
  task::sleep(50);
  
  //gyroTurn(-45, 20);
  //setIntake(100);
  //forwardPID(-30, 55); // go to center
  //turnPID(-44.15, 22, 1750); // turn to shoot
  turnPID(-43.75, 22, 1750); // turn to shoot
  forwardInches(-0.8, 13);
  setIntake(100);
  task::sleep(500);
  setIntake(0);
  task::sleep(100);
  
  R2BASE.spin(reverse, 100, pct); // shoot
  wait(600, msec);
  task pog(catapultReset);

  task::sleep(1000);
  // pick up to shoot again
  setIntake(100);
  gyroTurn(-10, 17);
  forwardInches(5, 17);
  forwardInches(-5, 17);
  
  gyroTurn(-75, 22);
  
  setIntake(100);
  forwardInches(7, 17);
  forwardInches(-6, 17);
  
  //turnPID(-44, 22, 2500); // turn to shoot
  turnPID(-44, 22, 2500); // turn to shoot
  setIntake(0);
  //forwardInches(-0.4, 12);

  R2BASE.spin(reverse, 100, pct); // shoot
  wait(600, msec);
  task m(catapultReset);

  stopBase();

}


void setonSkills()
{
  resetTotalDistance();  
  //pto6();

  //forwardInchesTimed(1.5, 5, 750);
  setBase(5); // move into rollers
  task::sleep(300);
  holdBase();

  roller(450,100);
  //setIntake(50);

          //wait(200, msec);
          //setIntake(0);
         //stopBase();
        
 // back up from roller and turn towards center of field
  //forwardPID(-2.8, 35);
  forwardInches(-1.5, 20);
  turnPID(138, 25);
  setIntake(100);
  forwardInches(6, 20);
  
  gyroTurn(145, 20);
  forwardInches(6, 20);
  turnPID(90, 26);
  //setIntake(0);

 // move into rollers
 
  setIntake(0);
  forwardInchesTimed(2.25, 15, 1500);
  setBase(5);
  task::sleep(275);
  holdBase();

  roller(475, 100);

  forwardInches(-1.5, 15);
  turnPID(-2.3, 27);
  setIntake(100);
  forwardPID(-26, 45);
  
  gyroTurn(0.5, 23);
  R2BASE.spin(reverse, 100, pct); // shootol
  wait(600, msec);
  task m(catapultReset);
  
  wait(100, msec);

  forwardInches(4, 20);
  
  turnPID(-60, 23);
  forwardPID(17, 30);
  setIntake(100);
  turnPID(-147, 27);

}

void pidTest()
{
  //forwardPID(10,50);
  turnPID(90, 40);
}

// DRIVER ///////////////////////////////////////////////////////////////////
void usercontrol(void)
{
  // set up controller callbacks befoe the code starts
  controllerPrim.ButtonR1.pressed(toggleLock);
  //controllerPrim.ButtonR1.pressed(catapultResetDriver); // fire in a separate task
  //controllerPrim.ButtonX.pressed(manspread); // deploy endgame mech

  L1BASE.setStopping(brakeType::coast);
  L2BASE.setStopping(brakeType::coast);
  L3BASE.setStopping(brakeType::coast);
  //L4BASE.setStopping(brakeType::coast);

  R1BASE.setStopping(brakeType::coast);
  R2BASE.setStopping(brakeType::coast);
  R3BASE.setStopping(brakeType::coast);
  //R4BASE.setStopping(brakeType::coast);

  // User control code here, inside the loop
  while (1 == 1) {

    L1BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    L2BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    L3BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    //L4BASE.spin(fwd, controllerPrim.Axis3.value(), pct);

    R1BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    R2BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    R3BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    //R4BASE.spin(fwd, controllerPrim.Axis2.value(), pct);

    // crossbow reload buttons
    if (controllerPrim.ButtonL1.pressing()) {
      XBOW.spin(forward, 100, pct); // backward just in case
    } else if (controllerPrim.ButtonL2.pressing()) {
      XBOW.spin(reverse, 100, pct); // forward, only one we really need
    } else {
      XBOW.stop();
    }


  // if (PTO.value() == pto_6m_val) // if power takeoff is out
    // {
    //   // catapult
    //   if (controllerPrim.ButtonR2.pressing()) {
    //     resettingCata = false; // stop resetting cata if manually pressed
    //     R2BASE.spin(reverse, 100, pct);
    //   }
    //   else if (!resettingCata) // if not currently rresetting, stop when not prressing manual cata
    //   {
    //     R2BASE.stop(coast);
    //   }
    //   /* else if (controllerPrim.ButtonR1.pressing()) {
    //     R2BASE.spin(reverse, 100, pct);
    //   } else {
    //     R2BASE.stop(coast);
    //   }*/

    //   // Rollers/intake
    //   if (controllerPrim.ButtonL1.pressing()) {
    //     L2BASE.spin(fwd, 100, pct); // intake
    //   } else if (controllerPrim.ButtonL2.pressing()) {
    //     L2BASE.spin(reverse, 100, pct); // outtake
    //   } else {
    //     L2BASE.stop(coast);
    //   }
    // }
    // else { // power takeoff in - all 8 motors. include 2
    //   resettingCata = false;
    //   L2BASE.spin(fwd, controllerPrim.Axis3.value(), pct);
    //   R2BASE.spin(fwd, controllerPrim.Axis2.value(), pct);
    // }

    /*
    // Tap the screen to move to a corresponding point on the field
    if (Brain.Screen.pressing())
    {
      setTarget(getScreenTouchX(), getScreenTouchY());
      //turnToTarget(30);
      moveToTarget(35, 20);
    }*/
  
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}



void values() {
  //Brain.Screen.setFillColor(color(10, 80, 30)); // Set background to green in rgb
  Brain.Screen.setPenColor(white); // Set text color to white

  Brain.Screen.printAt(210, 30, "Rot: %.1f deg       ", getRotationDeg());
  Brain.Screen.printAt(210, 50, "Dis: %.1f              ", getTotalDistance());
  Brain.Screen.printAt(210, 90, "%d  %d   ", (int)L1BASE.velocity(pct), (int)R1BASE.velocity(pct));
  Brain.Screen.printAt(210, 110, "%d  %d   ", (int)L2BASE.velocity(pct), (int)R2BASE.velocity(pct));
  Brain.Screen.printAt(210, 130, "%d  %d   ", (int)L3BASE.velocity(pct), (int)R3BASE.velocity(pct));

/*
  // Display debug values such as position, rotation, encoder values, total distancel, etc.
  //Brain.Screen.printAt(210, 30, "Pos: (%.1f, %.1f)     ", getGlobalX(), getGlobalY());
  Brain.Screen.printAt(210, 30, "Rot: %.1f deg      ", getRotationDeg());
  //Brain.Screen.printAt(210, 70, "Enc: L:%.1f R:%.1f    ", getLeftReading(), getRightReading());
  Brain.Screen.printAt(210, 50, "Dis: %.7f", getTotalDistance());
  */

  //Brain.Screen.printAt(210, 110, "PTO 8m: %d", PTO.value() ==  pto_8m_val);
  //Brain.Screen.printAt(210, 90, "Mtr %d", getTotalDistance());
  //Brain.Screen.printAt(210,110, "Rot: %.2f deg      ", encoderL.rotation(deg));
  //Brain.Screen.printAt(210, 130, "isStopped: %d   ", isStopped());

  /* Brain.Screen.printAt(210, 110, "lir: %.1f  aaom: %.1f", lastInertialRadians * toDegrees, absoluteAngleOfMovement * toDegrees);
  Brain.Screen.printAt(210, 130, "dth: %.1f  dd:%.1f", deltaTheta * toDegrees, deltaDistance);
  Brain.Screen.printAt(210, 150, "dl:%.1f  dr:%.1f", deltaLeft, deltaRight);
  Brain.Screen.printAt(210, 170, "dx:%.1f  dy:%.1f", deltaX, deltaY);*/
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(pidTest);
  Competition.drivercontrol(usercontrol);
  //newFunction();
  // Run the pre-autonomous function.
  pre_auton();

  //controllerPrim.ButtonDown.pressed(toggleopneumatic);

  // Prevent main from exiting with an infinite loop.
  // A lot of asyncronous tasks separate from the auton and driver task occur here

  //task c(catapultPID); // control the cata's PID separate from autonomous task


  while (true) {

    //catapultPID(); // control the cata's PID separate from autonomous task

    // Run these independently of auton and driver tasks
    // Show the debug values and the odometry display
    
    //updatePosition(); // Update the odometry position
    //odomDisplay();


    if (Brain.Screen.pressing())
    {
      INERTIAL.calibrate();
    }

    values();
    
    task::sleep(10); // Wait some time between odometry cycles. Test making it shorter for better position estimates
  }
}