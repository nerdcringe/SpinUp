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
  float fwd_error = 0;
  float fwd_lastError = 0;
  float fwd_integral = 0;
  float fwd_derivative = 0;

  float turn_error = 0;
  float turn_lastError = 0;
  float turn_integral = 0;
  float turn_derivative = 0;


  // ODOM VARS

  double lastForwardReading = 0;
  double lastSidewaysReading = 0;

  double lastRReading = 0;
  double lastLReading = 0;
  double lastSReading = 0;
  double lastRadians = 0;

  double globalX = 0;
  double globalY = 0;

  double targetX = 0;
  double targetY = 0;
  double targetDistance = 0; // current distance from target position
  double targetAngle = 0;

  // tracking wheel perpendicular distances from center
  const double leftOffset = 4.875; // left
  const double rightOffset = 4.875; // right
  const double sideOffset = -0.75; // sideways

  float lastLeftPos = 0;
  float lastRightPos = 0;
  float lastSidePos = 0;

  float deltaTheta = 0;
  float thetaNew = 0;

  float curLeft = 0;
  float curRight = 0;
  float curSide = 0;

  float leftAtReset = 0;
  float rightAtReset = 0;
  float thetaReset = 0;

  float deltaLeft = 0;
  float deltaRight = 0;
  //float deltaSide = 0;

  float deltaDistance;
  //float deltaDistanceSideways;

  // Accumulates the distance traveled during the current movement (replaces motor encoders)
  // Check if this is above a certain amount to start parallel tasks after moving a certain distance
  float totalDistance = 0;



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



  // OLD STUFF FOR MOTOR ENCODERS
  /*
  // Convert distance to move to ticks to rotate base motors
  double inchesToTicks(double inches)
  {
    return inches * (360 / WHEEL_CIRCUMFERENCE);
  }

  double ticksToInches(double ticks)
  {
    return ticks * (WHEEL_CIRCUMFERENCE / 360);
  }*/


  // Convert between inches and ticks of tracking wheels
  double inchesToTicks(double inches)
  {
    return inches * (360/ TRACKING_CIRCUMFERENCE);
  }

  double ticksToInches(double revs)
  {
    return revs * (TRACKING_CIRCUMFERENCE / 360);
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

  // Don't really need motor encoder functions as tracking wheels can do it more accurately and simpler
/*
  double getMotorPos()
  {
    double sum = LFBASE.rotation(deg)
              // + LM1BASE.rotation(deg)
              // + LM2BASE.rotation(deg)
              + LBBASE.rotation(deg)
              + RFBASE.rotation(deg)
              // + RM1BASE.rotation(deg)
              // + RM2BASE.rotation(deg)
              + RBBASE.rotation(deg);
    return ticksToInches(sum) / 4;
  }

  void clearMotorPos()
  {
    
    RFBASE.resetRotation();
    //RM1BASE.resetRotation();
    //RM2BASE.resetRotation();
    RBBASE.resetRotation();

    LFBASE.resetRotation();
    //LM1BASE.resetRotation();
    //LM2BASE.resetRotation();
    LBBASE.resetRotation();
  }*/


  double getTotalDistance()
  {
    return totalDistance;
  }

  void resetTotalDistance()
  {
    totalDistance = 0;
  }

 // Check if position is not changing so robot doesn't get stuck trying to drive past a wall (todo)
  bool isStopped()
  {
    return deltaDistance = 0;
  }


  double getRightReading() {
    return -ticksToInches(encoderR.rotation(deg)); // negative because right encoder is backwards
  }

  double getLeftReading() {
    return ticksToInches(encoderL.rotation(deg));
  }

  double getSideReading() {
  return ticksToInches(encoderS.rotation(deg));
  }



  double getDegrees()
  {
    return -INERTIAL.rotation(deg); // Important that this is negative
    // Positive angles are CCW (left) and negative angles are CW (right)
  }

  double getRadians()
  {
    return getDegrees()*toRadians;
  }


  // positive angles are left, negative is right. 0 degrees starts at positive x axis
  double getAngleToPosition(double x, double y)
  {
    double relativeX = x - globalX;
    double relativeY = y - globalY;

    // atan2 gives the angle to any position relative to the robot's position
    double angleToPosition = toDegrees * atan2(relativeY, relativeX);

    // Prevent the robot from targeting a rotation over 180 degrees from its current rotation.
    // If it's more than 180 it's faster to turn the other direction
    angleToPosition = angleWrap(angleToPosition, getDegrees());
    
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
      double currentForward = getRightReading();
      double currentSideways = getSideReading();
      double currentRadians = getRadians();

      // Get the change in encoder values since last update
      double forwardChange = currentForward - lastForwardReading;
      double sidewaysChange = currentSideways - lastSidewaysReading;
                                                                    //double rotationChange = currentRotation - lastRotation;

      // Calculate global position change due to going forward/backward
      // For example, this is what the forward calculation looks like
      

      // Accumulate each tiny change in position to the global position
      // Using trig, we can obtain the x and y components from the distance moved and current rotation
      globalX += forwardChange * cos(currentRadians);
      globalY += forwardChange * sin(currentRadians);

      // global position is offset by a few inches when rotated at certain angles
      // cancel out these offsets to keep tracking center stationary when rotating
      // position may still vary by 0.5 inches
      /*xOffset = 2.4 * sin(currentRadians);
      yOffset = -6 * (cos(currentRadians) - 1) * 0.5;*/

      // Position change due to moving sideways
      // Even if the robot doesn't drive sideways it still might jerk
      //globalX += sidewaysChange * sin(currentRotation * toRadians);
      //globalY += sidewaysChange * cos(currentRotation * toRadians);

      // Update the old values for the next update
      lastForwardReading = currentForward;
      lastSidewaysReading = currentSideways;
      //lastRadians = currentRotation;
  }



  // Run one odometry calculation to update global position
  // Run in a parallel task during auton
  void updatePosition()
  {
    // Get current values of encoders
    curLeft = getLeftReading();
    curRight = getRightReading();
    //curSide = getSideReading();

    // Get change in encoder values
    deltaLeft = (curLeft - lastLeftPos);
    deltaRight = (curRight - lastRightPos);
    //deltaSide = (curSide - lastSidePos);

    // Save the current encoder values to use for next update
    lastLeftPos = curLeft;
    lastRightPos = curRight;
    //lastSidePos = curSide;

    // Angle of arc of movement (different than inertial angle)
    // The center of the arc is an arbitrary distance away
    // The arc can be seen at the pilons document http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf (page 5)
    thetaNew = (curLeft - curRight) / (leftOffset + rightOffset);
    deltaTheta = (deltaLeft - deltaRight)/ (leftOffset + rightOffset);

    // If not turning deltaX is the distance moved forward
    if (deltaTheta == 0)
    {
      deltaDistance = (deltaRight + deltaLeft) / 2; // average of change in both encoders
      //deltaY = deltaSide;
    }
    else // adjust for the offset of right encoder from center using the arc
    {
      // Get the radius from the center of the arc to the center of the bot
      double centerRadius = deltaRight/deltaTheta + rightOffset; // Add right encoder offset from center of bot to the right encoder's radius

      // Get the chord length (straight line distance between ends of the arc) using 2r * sin(theta/2)
      deltaDistance = 2 * centerRadius * sin(deltaTheta/2);

      // Add the right encoder's offset to the right encoder's radius from the center of the arc to get the radius of the center of the bot
      // This radius is multiplied by 2sin(deltaTheta) is the length of the chord of the arc
      //deltaX = 2*sin(deltaTheta/2) * ((deltaRight/deltaTheta) + Sr);
      //deltaY = 2*sin(deltaTheta/2) * ((deltaSide/deltaTheta) + Ss);
    }
    
    // Accumulate the tiny change in position to the global position
    // Using trig, we can obtain the x and y components from the distance moved and current rotation (converting polar coordinates to x,y)
    double intertialRadians = getRadians();
    globalX += deltaDistance * cos(intertialRadians); // x uses cosine
    globalY += deltaDistance * sin(intertialRadians); // y uses sine
    
    //globalX += deltaDistanceSideways * sin(intertialRadians);
    //globalY += deltaDistanceSideways * cos(intertialRadians);

    totalDistance += deltaDistance; // accumulate the distance to the total distance
  }

  

  float odomFwdPID(double target, double maxSpeed)
  {
    float Kp = 1;
    float Ki = 0.005;
    float Kd = 0.175;
    float integralPowerLimit =
        40 / Ki;                   // little less than half power in pct (percent)
    float integralActiveZone = 15;
    float errorThreshold = 0.25; // Exit loop when error is less than this

    float speed = 0;

    fwd_error = target;

    if (fabs(fwd_error) > errorThreshold)
    {
      if (fabs(fwd_error) < integralActiveZone) {
        fwd_integral = fwd_integral + fwd_error;
      } else {
        fwd_integral = 0;
      }
      fwd_integral = keepInRange(fwd_integral, -integralPowerLimit, integralPowerLimit);

      fwd_derivative = fwd_error - fwd_lastError;

      speed = (Kp * fwd_error) + (Ki * fwd_integral) + (Kd * fwd_derivative);
      speed =  keepInRange(speed, -maxSpeed, maxSpeed);
      Brain.Screen.printAt(210, 140, "P: %.1f, I: %.1f, D: %.1f    ", (Kp * fwd_error) + (Ki * fwd_integral) + (Kd * fwd_derivative));
    }
    else
    {
      fwd_integral = 0;
      fwd_derivative = 0;
      speed = 0;
      Brain.Screen.printAt(210, 140, "PID Fwd Completed             ");
    }
    fwd_lastError = fwd_error;
    
    return speed;
  }


  float odomTurnPID(double target, double maxSpeed)
  {
    float Kp = 0.4;
    float Ki = 0.01;
    float Kd = 0.45;
    float integralPowerLimit =
        40 / Ki;                   // little less than half power in pct (percent)
    float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks
    float errorThreshold = 0.5; // Exit loop when error is less than this

    float speed = 0;


    turn_error = target - getDegrees();

    if (fabs(turn_error) > errorThreshold)
    {
      if (fabs(turn_error) < integralActiveZone) {
        turn_integral = turn_integral + turn_error;
      } else {
        turn_integral = 0;
      }
      turn_integral = keepInRange(turn_integral, -integralPowerLimit, integralPowerLimit);

      turn_derivative = turn_error - turn_lastError;

      speed = ((Kp * turn_error) + (Ki * turn_integral) + (Kd * turn_derivative));
      speed =  keepInRange(speed, -maxSpeed, maxSpeed);
      Brain.Screen.printAt(210, 160, "P: %.1f, I: %.1f, D: %.1f    ", (Kp * turn_error), (Ki * turn_integral), (Kd * turn_derivative));
    }
    else
    {
      turn_integral = 0;
      turn_derivative = 0;
      speed = 0;
      Brain.Screen.printAt(210, 160, "PID Turn Completed             ");
    }
    turn_lastError = turn_error;

    return speed;
  }

  
  float odomDriveStraightPID(double target, double maxSpeed)
  {
    float Kp = 0.3;
    float Ki = 0.0;
    float Kd = 0.3;
    float integralPowerLimit =
        40 / Ki;                   // little less than half power in pct (percent)
    float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks
    float errorThreshold = 0.3; // Exit loop when error is less than this

    float speed = 0;

    turn_error = target - getDegrees();

    if (fabs(turn_error) > errorThreshold)
    {
      if (fabs(turn_error) < integralActiveZone) {
        turn_integral = turn_integral + turn_error;
      } else {
        turn_integral = 0;
      }
      turn_integral = keepInRange(turn_integral, -integralPowerLimit, integralPowerLimit);

      turn_derivative = turn_error - turn_lastError;

      speed = ((Kp * turn_error) + (Ki * turn_integral) + (Kd * turn_derivative));
      speed =  keepInRange(speed, -maxSpeed, maxSpeed);
      Brain.Screen.printAt(210, 160, "P: %.1f, I: %.1f, D: %.1f       ", (Kp * turn_error), (Ki * turn_integral), (Kd * turn_derivative));
    }
    else
    {
      turn_integral = 0;
      turn_derivative = 0;
      speed = 0;
      Brain.Screen.printAt(210, 160, "PID Drive Straight Completed             ");
    }
    turn_lastError = turn_error;

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
    
    Brain.Screen.printAt(210, 140, "turnTo() done.                    ");
  }


  

  void moveTo(double x, double y, double maxFwdSpeed, double maxTurnSpeed)
  {
    resetTotalDistance();

    // Current components of final speed to be summed up
    double curFwdSpeed = 1;
    double curTurnSpeed = 1;

    targetX = x;
    targetY = y;
    targetDistance = distanceTo(targetX, targetY, globalX, globalY);
    
    bool followPosition = true;
    double lastDistToGo = 0; // Keeps track of the last couple inches to finish off the movement

    // Run while forward pid is active (speed > 0) SUGGESTION: make error threshold here instead of in PID function
    while (curFwdSpeed != 0)
    {
      // When close to target, don't follow the position, just go forward
      // so the robot doesn't run in circles trying to pinpoint the exact position
      if (fabs(targetDistance) < 2)
      {
        followPosition = false;
      }

      if (followPosition)
      {

        targetDistance = distanceTo(targetX, targetY, globalX, globalY);
        targetAngle = getAngleToPosition(targetX, targetY);
        
        curFwdSpeed = odomFwdPID(targetDistance, maxFwdSpeed); // calculate pid forward speed

        // When needing to turn a lot, use turn pid
        if (fabs( targetAngle - getDegrees() ) > 3)
        {
          curTurnSpeed = odomTurnPID(targetAngle, maxTurnSpeed);
        }
        else  // when maintaining current angle, use driveStraight PID
        {
          curTurnSpeed = odomDriveStraightPID(targetAngle, maxTurnSpeed); // limit max speed here
        }

        lastDistToGo = getTotalDistance(); // keep track of the last distance to go until robot stops following position

      }
      else // when close to target, just move forward the last little distance and don't turn
      {
        double desiredDistance = lastDistToGo + targetDistance; // add the last inch or so to the desired distance
        curFwdSpeed = odomFwdPID(desiredDistance - getTotalDistance(), maxFwdSpeed); // error = desired - actual distance
        curTurnSpeed = 0;

      }


      leftDrive(curFwdSpeed - curTurnSpeed);
      rightDrive(curFwdSpeed + curTurnSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetAngle);
      task::sleep(5);
    }

    Brain.Screen.printAt(210, 120, "moveTo() done.                    ");
  }




  void moveToRev(double x, double y, double maxFwdSpeed, double maxTurnSpeed)
  {
    resetTotalDistance();

    // Current components of final speed to be summed up
    double curFwdSpeed = 1;
    double curTurnSpeed = 1;

    targetX = x;
    targetY = y;
    targetDistance = distanceTo(targetX, targetY, globalX, globalY);
    
    bool followPosition = true;
    double lastDistToGo = 0; // Keeps track of the last couple inches to finish off the movement

    // Run while forward pid is active
    while (curFwdSpeed != 0)
    {
      // When close to target, don't follow the position, just go forward
      // so the robot doesn't run in circles trying to pinpoint the exact position
      if (fabs(targetDistance) < 2)
      {
        followPosition = false;
      }

      if (followPosition)
      {
      // set the target global variables to reflect the given parameters
        targetDistance = -distanceTo(targetX, targetY, globalX, globalY); // distance is negative

        targetAngle = getAngleToPosition(targetX, targetY);
        targetAngle = angleWrap(targetAngle - 180, getDegrees()); // angle is 180 degrees so it faces backwards

        curFwdSpeed = odomFwdPID(targetDistance, maxFwdSpeed); // calculate pid forward speed

        // When needing to turn a lot, use turn pid
        if (fabs( targetAngle - getDegrees() ) > 3)
        {
          curTurnSpeed = odomTurnPID(targetAngle, maxTurnSpeed);
        }
        else  // when maintaining current angle, use driveStraight PID
        {
          curTurnSpeed = odomDriveStraightPID(targetAngle, maxTurnSpeed); // limit max speed here
        }

        lastDistToGo = getTotalDistance(); // use as the initial position of final forward
      }
      else // when close to target, just move forward the last little distance and don't turn
      {
        curFwdSpeed = odomFwdPID(lastDistToGo + targetDistance - getTotalDistance(), maxFwdSpeed);
        curTurnSpeed = 0;
      }

      leftDrive(curFwdSpeed - curTurnSpeed);
      rightDrive(curFwdSpeed + curTurnSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetAngle);
      task::sleep(5);
    }

    Brain.Screen.printAt(210, 120, "moveTo() done.                    ");
  }
  

  // Waypoints are a point the robot follows but doesn't stop at
  // Waypoints allow more control over the path of the robot
  void moveToWaypoint(double x, double y, double maxFwdSpeed, double maxTurnSpeed)
  {
    resetTotalDistance();

    // Current components of final speed to be summed up
    double curTurnSpeed = 1;

    double initialX = globalX;
    double initialY = globalY;


    targetX = x;
    targetY = y;
    targetDistance = distanceTo(targetX, targetY, globalX, globalY);
    
    bool passedX = false;
    bool passedY = false;
    
    // run while the robot has not passed the point's x and y position yet
    while (!passedX || !passedY)
    {
      // Check if the point has been passed yet
      passedX = (initialX > x && globalX < x) || (initialX < x && globalX > x);
      passedY = (initialY > y && globalY < y) || (initialY < y && globalY > y);

      targetDistance = distanceTo(targetX, targetY, globalX, globalY);

      // only turn when far away
      if (fabs(targetDistance) > 2)
      {
        targetAngle = getAngleToPosition(targetX, targetY);
        
        // When needing to turn a lot, use turn pid
        if (fabs( targetAngle - getDegrees() ) > 3)
        {
          curTurnSpeed = odomTurnPID(targetAngle, maxTurnSpeed);
        }
        else  // when maintaining current angle, use driveStraight PID
        {
          curTurnSpeed = odomDriveStraightPID(targetAngle, maxTurnSpeed); // limit max speed here
        }
      }
      else // stop turning when close
      {
        curTurnSpeed = 0;
      }

      leftDrive(maxFwdSpeed - curTurnSpeed);
      rightDrive(maxFwdSpeed + curTurnSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetAngle);
      task::sleep(5);
    }

    Brain.Screen.printAt(210, 120, "moveToWaypoint() done.                    ");
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

    Brain.Screen.setFillColor(color(10, 80, 30)); // green in rgb
    Brain.Screen.setPenColor(white);
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
  Brain.Screen.printAt(210, 90, "Dist: %.1f", getTotalDistance());
  //Brain.Screen.printAt(210, 90, "Mot: L: %.1f R: %.1f    ", getLeftMotors(), getRightMotors());
}

int backgroundTasks()
{
  while (true)
  {
    updatePosition();
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
  Brain.Screen.setPenColor(white);

  task a(backgroundTasks);
  
  
  moveToWaypoint(10, 10, 30, 40);
  moveTo(20, 20, 30, 40);
  // final angles: 40.3, 39.3, 39.3, 37.4, 39.1 ,38.8, 36.7
  // may need to turn to a specific angle or point afterwards to aim
  /*task::sleep(500);
  turnTo(30, 30, 22);*/


  /*moveTo(30, 10, 20, 20);
  moveToRev(0, -10, 20, 30);
  moveTo(0, 0, 20, 20);*/

  /*
  turnTo(10, 10, 35);
  moveTo(10, 10, 25, 20); // x, y, fwdSpeed, turnSpeed

  turnTo(20, 0, 35);
  moveTo(20, 0, 25, 20); // x, y, fwdSpeed, turnSpeed
*/

  /*moveTo(-20, 0, 25, 20);
  moveTo(-20, 20, 25, 20);*/

  while (1)
  {

    if (Brain.Screen.pressing())
    {
      targetX = screenToGlobalX(Brain.Screen.xPosition());
      targetY = screenToGlobalY(Brain.Screen.yPosition());
      //turnTo(targetX, targetY, 20);
      moveTo(targetX, targetY, 20, 25);
    }
    

    if (controllerPrim.ButtonUp.pressing())
    {
      drive(15);
    }
    else if (controllerPrim.ButtonDown.pressing())
    {
      drive(-15);
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
