/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\joshu                                            */
/*    Created:      Sun Jun 05 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;


// VARIABLES ////////////////////////////////

  #define PI 3.14159265
  const double toRadians = PI / 180.0; // multiply degrees by this
  const double toDegrees = 180.0 / PI;
  const double WHEEL_CIRCUMFERENCE = 3.25 * PI;
  const double TRACKING_CIRCUMFERENCE = 2.75 * PI;

  // PID VARS
  float errorFwd = 0;
  float lastErrorFwd = 0;
  float integralFwd = 0;
  float derivativeFwd = 0;

  float errorTurn = 0;
  float lastErrorTurn = 0;
  float integralTurn = 0;
  float derivativeTurn = 0;


  // ODOM VARS
  double lastForwardReading = 0;
  double lastSidewaysReading = 0;

  double lastRReading = 0;
  double lastLReading = 0;
  double lastSReading = 0;
  double lastRadians = 0;


  double globalX = 0;
  double globalY = 0;

  double xOffset = 0;
  double yOffset = 0;


  double targetX = 0;
  double targetY = 0;
  double targetDistance = 0; // current distance from target position
  double targetAngle = 0;

  double lastDistToGo = 0; // how much to go forward for the last couple inches of the movement

  
  const double TL = 4.875; // left tracking wheel perpendicular distance from center
  const double TR = 4.875; // right tracking wheel perpendicular distance from center
  const double TB = -0.75; // back tracking wheel perpendicular distance from center
  



  const double Sl = 4.875; // left tracking wheel perpendicular distance from center
  const double Sr = 4.875; // right tracking wheel perpendicular distance from center
  const double Ss = -0.75; // back tracking wheel perpendicular distance from center



  double lastTheta = 0;



  float lastLeftPos = 0;
  float lastRightPos = 0;
  float lastSidePos = 0;

  float deltaTheta = 0;
  float thetaNew = 0;
  float thetaM = 0;


  float curLeft = 0;
  float curRight = 0;
  float curSide = 0;

  float leftAtReset = 0;
  float rightAtReset = 0;
  float thetaReset = 0;

  float deltaLeft = 0;
  float deltaRight = 0;
  float deltaSide = 0;

  //float deltaLr = 0;
  //float deltaRr = 0;

  float deltaX = 0;
  float deltaY = 0;


  float theta = 0;
  float radius = 0;





// DEVICES ///////////////////////////////////

  motor LFBASE(PORT1, false); // for new bot port 15 && for old bot port 11
  //motor LMBASE(PORT2); // Left mid
  motor LBBASE(PORT19, false);

  motor RFBASE(PORT13, true);       // true for old bot
  //motor RMBASE(PORT14,true); // Right mid
  motor RBBASE(PORT18, true);       // true for old bot



  inertial INERTIAL(PORT14);

  triport Triport(PORT22); // Get reference for three-wire ports on brain
  encoder encoderL(Triport.A); // left tracking wheel
  encoder encoderR(Triport.C); // right tracking wheel
  encoder encoderS(Triport.F); // sideways tracking wheel. Testing to see if we need this to deal with drift


  controller controllerPrim(controllerType::primary);



// MATH FUNCTIONS /////////////////////////////////////////////



  // Convert distance to move to ticks to rotate base motors
  double inchesToTicks(double inches)
  {
    return inches * (360 / WHEEL_CIRCUMFERENCE);
  }

  double ticksToInches(double ticks)
  {
    return ticks * (WHEEL_CIRCUMFERENCE / 360);
  }


  // Convert between inches and revolutions of tracking wheels
  // Revolutions are simpler than degrees for non-motor encoders
  double inchesToRevs(double inches)
  {
    return inches / TRACKING_CIRCUMFERENCE;
  }

  double revsToInches(double revs)
  {
    return revs * TRACKING_CIRCUMFERENCE;
  }





  double keepInRange(double n, double bottom, double top) {
    if (n < bottom)
      n = bottom;
    if (n > top)
      n = top;
    return n;
  }


  double distanceTo(double x1, double y1, double x2, double y2)
  {
      return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
  }


  // Keep an angle in -180 to 180 range
  double keepAngleIn180(double a)
  {
    while(a > 180)
      {
        a -= 360;
      }
      while(a < -180)
      {
        a += 360;
      }
      return a;
  }


  // Keep rotation within 180 degrees of the reference (robot) so it doesnt turn too much
  double angleWrap(double a, double referenceAngle)
  {
    while(a > referenceAngle + 180) // Subtract 360 if angle is too large
    {
      a -= 360;
    }
    while(a <= referenceAngle - 180) // Add 360 if angle is too large
    {
      a += 360;
    }

    return a;
  }




// SENSOR FUNCTIONS ///////////////////////////////////////////////////////

  double getMotorEncoders()
  {
    double sum = LFBASE.rotation(rev)
              // + LM1BASE.rotation(deg)
              // + LM2BASE.rotation(deg)
              + LBBASE.rotation(rev)
              + RFBASE.rotation(rev)
              // + RM1BASE.rotation(deg)
              // + RM2BASE.rotation(deg)
              + RBBASE.rotation(rev);
    return revsToInches(sum) / 4;
  }


  void clearMotorEncoders()
  {
    
    RFBASE.resetRotation();
    //RM1BASE.resetRotation();
    //RM2BASE.resetRotation();
    RBBASE.resetRotation();

    LFBASE.resetRotation();
    //LM1BASE.resetRotation();
    //LM2BASE.resetRotation();
    LBBASE.resetRotation();
  }


  double getForwardReading() {
    double revs = (encoderL.rotation(rev) + encoderR.rotation(rev)) / 2;
    return revsToInches(revs);
  }
  double getSidewaysReading() {
    double revs = encoderS.rotation(rev);
    return revsToInches(revs);
  }


  double getRightReading() {
    /*double ticks = ( RFBASE.rotation(deg) + RBBASE.rotation(deg) ) / 2;
    return ticksToInches(ticks);*/
    return -revsToInches(encoderR.rotation(rev));;
  }

  double getLeftReading() {
    return revsToInches(encoderL.rotation(rev));
    /*
    double ticks = ( LFBASE.rotation(deg) + LBBASE.rotation(deg) ) / 2;
    return ticksToInches(ticks);*/
  }

  double getSideReading() {
    //return ticksToInches(encoderB.rotation(deg));
  return revsToInches(encoderS.rotation(rev));
  }

/*
  double getRightMotors() {
    double ticks = ( RFBASE.rotation(deg) + RBBASE.rotation(deg) ) / 2;
    return ticksToInches(ticks);
    //encoderR.rotation(deg);//revsToInches(encoderR.rotation(rev));;
  }

  double getLeftMotors() {
    //return revsToInches(encoderL.rotation(rev));
    
    double ticks = ( LFBASE.rotation(deg) + LBBASE.rotation(deg) ) / 2;
    return ticksToInches(ticks);
  }
*/


  double getDegrees()
  {
    return -INERTIAL.rotation(deg); // Important that this is negative
    // Positive angles are CCW (left) and negative angles are CW (right)
  }

  double getRadians()
  {
    return getDegrees()*toRadians;
  }


  double getAngleToPosition(double x, double y)
  {
    double relativeX = x - globalX; // error = desired - actual
    double relativeY = y - globalY;
    
    double currentAngle = getDegrees();

    // atan2 gives the angle to any position relative to the robot
    double angleToPosition = toDegrees * atan2(relativeY, relativeX);

    // Prevent the robot from targeting a rotation over 180 degrees from its current rotation.
    // If it's more than 180 it's faster to turn the other direction
    angleToPosition = angleWrap(angleToPosition, currentAngle);
    return angleToPosition;
  }


// MOVEMENT FUNCTIONS ////////////////////////////
  void leftDrive(double power)
  {
    LFBASE.spin(fwd, power, pct);
    //LMBASE.spin(fwd, power, pct);
    LBBASE.spin(fwd, power, pct);
  }

  void rightDrive(double power)
  {
    RFBASE.spin(fwd, power, pct);
    //RMBASE.spin(fwd, power, pct);
    RBBASE.spin(fwd, power, pct);
  }

  void drive(double power)
  {
    leftDrive(power);
    rightDrive(power);
  }


  void stopBase()
  {
    LFBASE.stop(coast);
    //LMBASE.stop(coast);
    LBBASE.stop(coast);

    RFBASE.stop(coast);
    //RMBASE.stop(coast);
    RBBASE.stop(coast);
  }




// PID //////////////////////////////////////////////////////////////////////



// ODOMETRY /////////////////////////////////////////////////////////////////////////////

  // Run one odometry calculation to update global position
  // Run in a parallel task during auton
  void updatePositionOld()
  {
      // Get the current encoder distances and rotation 
      double currentForward = getForwardReading();
      double currentSideways = getSidewaysReading();
      double currentRadians = getRadians();

      // Get the change in encoder values since last update
      double forwardChange = currentForward - lastForwardReading;
      double sidewaysChange = currentSideways - lastSidewaysReading;
                                                                    //double rotationChange = currentRotation - lastRotation;

      // Calculate global position change due to going forward/backward
      // For example, this is what the forward calculation looks like
      
      /*
                          new
                        position
                          |
                          V
                        / |
          Forward    /   |
          change   /     | change in y
                  /       |
                /         |
              /           |
            /            _|
          /_____________|_|
          ^   change in x
          |
        current
        position

      */

      // Accumulate each tiny change in position to the global position
      // Using trig, we can obtain the x and y components from the distance moved and current rotation
      globalX += forwardChange * cos(currentRadians);
      globalY += forwardChange * sin(currentRadians);

      // global position is offset by a few inches when rotated at certain angles
      // cancel out these offsets to keep tracking center stationary when rotating
      // position may still vary by 0.5 inches
      xOffset = 2.4 * sin(currentRadians);
      yOffset = -6 * (cos(currentRadians) - 1) * 0.5;

      // Position change due to moving sideways
      // Even if the robot doesn't drive sideways it still might jerk
      //globalX += sidewaysChange * sin(currentRotation * toRadians);
      //globalY += sidewaysChange * cos(currentRotation * toRadians);

      // Update the old values for the next update
      lastForwardReading = currentForward;
      lastSidewaysReading = currentSideways;
      //lastRadians = currentRotation;
  }


  void updatePositionGood()
  {

    curLeft = getLeftReading();
    curRight = getRightReading(); //step 1
    curSide = getSideReading();

    deltaLeft = (curLeft - lastLeftPos);
    deltaRight = (curRight - lastRightPos);
    deltaSide = (curSide - lastSidePos);

    lastLeftPos = curLeft;
    lastRightPos = curRight; //step 3
    lastSidePos = curSide;

    double intertialRadians = getRadians();

    // Angle of arc of movement (different than inertial angle)
    thetaNew = (curLeft - curRight) / (Sl + Sr);
    deltaTheta = (deltaLeft - deltaRight)/ (Sl + Sr);

      
    if (deltaTheta == 0)
    {
      deltaX = (deltaRight + deltaLeft) / 2;
      //deltaY = deltaSide;
      Brain.Screen.printAt(200, 180, "deltaTheta: %f", deltaX);
    }
    else
    {
      deltaX = 2*sin(deltaTheta/2) * ((deltaRight/deltaTheta) + Sr);
      //deltaY = 2*sin(deltaTheta/2) * ((deltaSide/deltaTheta) + Ss); //step 8
    }
    
    globalX += deltaX * cos(intertialRadians);
    globalY += deltaX * sin(intertialRadians);
    
    //globalX += deltaY * sin(intertialRadians);
    //globalY += deltaY * cos(intertialRadians);

    lastTheta = thetaNew;
  }

  


  float odomFwdPID(double target, double maxSpeed)
  {
    float Kp = 1;
    float Ki = 0.005;
    float Kd = 0.15;
    float integralPowerLimit =
        40 / Ki;                   // little less than half power in pct (percent)
    float integralActiveZone = 15;
    float errorThreshold = 0.5; // Exit loop when error is less than this

    float speed = 0;

    errorFwd = target;

    if (fabs(errorFwd) > errorThreshold)
    {
      if (fabs(errorFwd) < integralActiveZone) {
        integralFwd = integralFwd + errorFwd;
      } else {
        integralFwd = 0;
      }
      integralFwd = keepInRange(integralFwd, -integralPowerLimit, integralPowerLimit);

      derivativeFwd = errorFwd - lastErrorFwd;

      speed = (Kp * errorFwd) + (Ki * integralFwd) + (Kd * derivativeFwd);
      speed =  keepInRange(speed, -maxSpeed, maxSpeed);
      Brain.Screen.printAt(210, 140, "P: %.1f, I: %.1f, D: %.1f    ", (Kp * errorFwd) + (Ki * integralFwd) + (Kd * derivativeFwd));
    }
    else
    {
      integralFwd = 0;
      derivativeFwd = 0;
      speed = 0;
      Brain.Screen.printAt(210, 140, "PID Fwd Completed             ");
    }
    lastErrorFwd = errorFwd;
    
    return speed;
  }


  float odomTurnPID(double target, double maxSpeed)
  {
    float Kp = 0.4;
    float Ki = 0.02;
    float Kd = 0.2;
    float integralPowerLimit =
        40 / Ki;                   // little less than half power in pct (percent)
    float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks
    float errorThreshold = 0.75; // Exit loop when error is less than this

    float speed = 0;


    errorTurn = target - getDegrees();

    if (fabs(errorTurn) > errorThreshold)
    {
      if (fabs(errorTurn) < integralActiveZone) {
        integralTurn = integralTurn + errorTurn;
      } else {
        integralTurn = 0;
      }
      integralTurn = keepInRange(integralTurn, -integralPowerLimit, integralPowerLimit);

      derivativeTurn = errorTurn - lastErrorTurn;

      speed = ((Kp * errorTurn) + (Ki * integralTurn) + (Kd * derivativeTurn));
      speed =  keepInRange(speed, -maxSpeed, maxSpeed);
      Brain.Screen.printAt(210, 160, "P: %.1f, I: %.1f, D: %.1f    ", (Kp * errorTurn), (Ki * integralTurn), (Kd * derivativeTurn));
    }
    else
    {
      integralTurn = 0;
      derivativeTurn = 0;
      speed = 0;
      Brain.Screen.printAt(210, 160, "PID Turn Completed             ");
    }
    lastErrorTurn = errorTurn;

    return speed;
  }

  
  float odomDriveStraightPID(double target, double maxSpeed)
  {
    float Kp = 0.2;
    float Ki = 0.02;
    float Kd = 0.1;
    float integralPowerLimit =
        40 / Ki;                   // little less than half power in pct (percent)
    float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks
    float errorThreshold = 0.75; // Exit loop when error is less than this

    float speed = 0;

    errorTurn = target - getDegrees();

    if (fabs(errorTurn) > errorThreshold)
    {
      if (fabs(errorTurn) < integralActiveZone) {
        integralTurn = integralTurn + errorTurn;
      } else {
        integralTurn = 0;
      }
      integralTurn = keepInRange(integralTurn, -integralPowerLimit, integralPowerLimit);

      derivativeTurn = errorTurn - lastErrorTurn;

      speed = ((Kp * errorTurn) + (Ki * integralTurn) + (Kd * derivativeTurn));
      speed =  keepInRange(speed, -maxSpeed, maxSpeed);
      Brain.Screen.printAt(210, 160, "P: %.1f, I: %.1f, D: %.1f       ", (Kp * errorTurn), (Ki * integralTurn), (Kd * derivativeTurn));
    }
    else
    {
      integralTurn = 0;
      derivativeTurn = 0;
      speed = 0;
      Brain.Screen.printAt(210, 160, "PID Drive Straight Completed             ");
    }
    lastErrorTurn = errorTurn;

    return speed;
  }



  void turnTo(double x, double y, double turnSpeed)
  {
    double finalTurnSpeed = 1;

    while (finalTurnSpeed != 0)
    { 
      targetX = x;
      targetY = y;
      targetAngle = getAngleToPosition(targetX, targetY);
      finalTurnSpeed = odomTurnPID(targetAngle, turnSpeed);
      
      leftDrive(-finalTurnSpeed);
      rightDrive(finalTurnSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetAngle);

      task::sleep(5);
    }
    
  }



  void moveTo(double x, double y, double fwdSpeed, double turnSpeed)
  {
    double finalFwdSpeed = 1;
    double finalTurnSpeed = 1;


    // Run while both forward and turn PIDs are active
    while (finalFwdSpeed != 0 || finalTurnSpeed != 0)
    {
      // set the target global variables to reflect the given parameters
      targetX = x;
      targetY = y;
      targetDistance = distanceTo(targetX, targetY, globalX, globalY);

      // Only update the angle and distance when far away
      // so the robot doesn't run in circles trying to pinpoint the exact location
      if (targetDistance > 2)
      {
        targetAngle = getAngleToPosition(targetX, targetY);

        finalFwdSpeed = odomFwdPID(targetDistance, fwdSpeed);
        finalTurnSpeed = odomDriveStraightPID(targetAngle, turnSpeed);

        // keep track of last distance to go 
        lastDistToGo = targetDistance;
        clearMotorEncoders();
        //Brain.Screen.printAt(210, 180, "FAR %.1f    ", errorFwd);
      }
      else
      {
        // just move forward the last little distance and maintain the last angle given

        finalFwdSpeed = odomFwdPID(lastDistToGo - getMotorEncoders(), fwdSpeed);
        finalTurnSpeed = 0;
        //finalTurnSpeed = odomTurnPID(targetAngle, turnSpeed);
        //Brain.Screen.printAt(210, 190, "CLOSE %.1f    ", errorFwd);
      }


      /*if (errorTurn > 180) // slow down fwd if need to turn a lot
      {
        finalFwdSpeed *= 0.75f;
      }*/
      
      leftDrive(finalFwdSpeed - finalTurnSpeed);
      rightDrive(finalFwdSpeed + finalTurnSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetAngle);
      task::sleep(5);
    }

    Brain.Screen.printAt(210, 120, "Done moving                    ");
  }



// DISPLAY ///////////////////////////


  // screen is 480 x 272
  void drawPoint(double x, double y)
  {
      double tile_size = 30;
      double draw_pos_x = ((x / 24) * tile_size) + 105;
      double draw_pos_y = ((-y / 24) * tile_size) + 105;
      
      Brain.Screen.setFillColor(purple);
      Brain.Screen.setPenWidth(0);
      Brain.Screen.drawCircle(draw_pos_x, draw_pos_y, 4);
  }


  // Convert where the screen is tapped to field coordinates
  double screenToGlobalX(double screenX)
  {
      double tile_size = 30;
      return ((screenX - 105) / tile_size) * 24;;
  }

  // Convert where the screen is tapped to field coordinates
  double screenToGlobalY(double screenY)
  {
      double tile_size = 30;

      return -((screenY - 105) / tile_size) * 24;
  }



  // Draw the dashboard to visually display the robot's location and position
  void draw()
  {
      // Draw grid of the field
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(color(100, 100, 100));
      Brain.Screen.setPenWidth(2);

      int tile_size = 30;

      for (int x = 0; x < 6; x++)
      {
        for (int y = 0; y < 6; y++)
        {
          Brain.Screen.drawRectangle(x * tile_size + 15, y * tile_size + 15, tile_size, tile_size); // fill entire screen
        }
      }
      /*Brain.Screen.printAt(170, 100, "+x");
      Brain.Screen.printAt(20, 100, "-x");
      Brain.Screen.printAt(110, 30, "+y");
      Brain.Screen.printAt(110, 190, "-y");*/
      
      // Draw robot
      Brain.Screen.setFillColor(white);
      Brain.Screen.setPenWidth(0);
      double draw_pos_x = ((globalX/24) * tile_size) + 105;
      double draw_pos_y = ((-globalY/24) * tile_size) + 105; // make y negative because down is positive on the screen

      // Position of circle indicates the position found by odometry
      Brain.Screen.drawCircle(draw_pos_x, draw_pos_y, 6);
      Brain.Screen.setPenColor(red);
      Brain.Screen.setPenWidth(5);

      // Line indicates the rotation found by the gyro
      Brain.Screen.drawLine(draw_pos_x,
                            draw_pos_y,
                            draw_pos_x + cos(getDegrees() * toRadians) * 20,
                            draw_pos_y - sin(getDegrees() * toRadians) * 20); // make y negative because down is positive on the screen
      drawPoint(targetX, targetY);
  }



void debug()
{
  
    Brain.Screen.setFillColor(color(10, 80, 30)); // green in rgb
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(210, 30, "Pos: (%.1f, %.1f)     ", globalX, globalY);
    Brain.Screen.printAt(210, 50, "Rot: %.1f deg      ", getDegrees());
    Brain.Screen.printAt(210, 70, "Enc: L: %.1f R: %.1f S: %.1f    ", getLeftReading(), getRightReading(), getSideReading());
    //Brain.Screen.printAt(210, 90, "Mot: L: %.1f R: %.1f    ", getLeftMotors(), getRightMotors());
}

int backgroundTasks()
{
  while (true)
  {
    updatePositionGood();
    debug();
    draw();
    task::sleep(10);
  }
  return 0;
}



int main()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  Brain.Screen.setFillColor(color(10, 80, 30)); // green in rgb
  Brain.Screen.setPenWidth(0);
  Brain.Screen.drawRectangle(0, 0, 480, 272); // fill entire screen


  while (INERTIAL.isCalibrating())
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(210, 50, "Rot: CALLIBRATING");
  }

  task a(backgroundTasks);

  //leftDrive(-8);
  //rightDrive(8);

  /*moveTo(24, 0, 25, 15);
  turnTo(0, 0, 30);
  moveTo(0, 0, 25, 20);*/

  /*
  turnTo(10, 10, 35);
  moveTo(10, 10, 25, 20); // x, y, fwdSpeed, turnSpeed

  turnTo(20, 0, 35);
  moveTo(20, 0, 25, 20); // x, y, fwdSpeed, turnSpeed
*/
  while (1)
  {
    

    if (Brain.Screen.pressing())
    {
      targetX = screenToGlobalX(Brain.Screen.xPosition());
      targetY = screenToGlobalY(Brain.Screen.yPosition());
      //turnTo(targetX, targetY, 20);
    }
    
    /*turnToPoint(10);

      Brain.Screen.printAt(210, 120, "Desired: (%.1f, %.1f)", targetX, targetY);

      updateFwdPID();
      updateTurnPID();*/

      /*double driveSpeed = 0;
      if (controllerPrim.ButtonUp.pressing())
      {
        driveSpeed = 10;
      }
      else if (controllerPrim.ButtonDown.pressing())
      {
        driveSpeed = -10;
      }

      leftDrive(driveSpeed + finalFwdPower + finalTurnPower);
      rightDrive(driveSpeed + finalFwdPower - finalTurnPower);*/



      if (controllerPrim.ButtonUp.pressing())
      {
        drive(10);
      }
      else if (controllerPrim.ButtonDown.pressing())
      {
        drive(-10);
      }
      else if (controllerPrim.ButtonLeft.pressing())
      {
        leftDrive(-10);
        rightDrive(10);
      }
      else if (controllerPrim.ButtonRight.pressing())
      {
        leftDrive(10);
        rightDrive(-10);
      }
      else
      {
        stopBase();
      }

    task::sleep(5);
  }
}
