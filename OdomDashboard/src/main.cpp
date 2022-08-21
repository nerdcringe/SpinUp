/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Program:      OdomDashboard                                             */
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
  const double toRadians = PI / 180.0; // multiply degrees by this to convert to radians
  const double toDegrees = 180.0 / PI; // multiply radians by this to convert to degrees
  const double WHEEL_CIRCUMFERENCE = 3.25 * PI; // Circumferebce of the powered wheels (diameter * PI)
  const double TRACKING_CIRCUMFERENCE = 2.75 * PI; // Circumferebce of the tracking wheels (diameter * PI)


  // PID VARIABLES //
  float fwd_error = 0; // Distance from the target forward distance
  float fwd_lastError = 0;
  float fwd_integral = 0; // integral accumulates the error, speeding up if target is not reached fast enough
  float fwd_derivative = 0; // Derivative smooths out any oscillations at the end of the movement

  float turn_error = 0; // Relative angle from the target rotation (degrees)
  float turn_lastError = 0;
  float turn_integral = 0;
  float turn_derivative = 0;


  // ODOMETRY VARIABLES //

  // *** Current global position on the field ***
  // These values always start at (0, 0) so everything is relative to the starting location
  double globalX = 0;
  double globalY = 0;

  double absoluteAngleOfMovement = 0;

  // tracking wheel perpendicular distances from center.
  // Seen at page 4 of http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
  const double leftOffset =  4.875; // left
  const double rightOffset = 4.875; // right
  const double sideOffset = 0.25;//-0.75; // sideways

  // Current value of tracking wheel encoder positions
  float curLeft = 0;
  float curRight = 0;
  float curSide = 0;

  // Values of tracking wheel encoder positions and inertial radians since last update
  float lastLeftPos = 0;
  float lastRightPos = 0;
  float lastSidePos = 0;
  double lastInertialRadians = 0;

  // Change in tracking wheel encoder positions since last update
  float deltaLeft = 0;
  float deltaRight = 0;
  float deltaSide = 0;

  // Change in arc theta (twice the change in inertial radians) since last update
  float deltaTheta = 0;

  // Change in global position since last update
  double deltaX = 0;
  double deltaY = 0;

  // Distance moved relative to the bot's direction
  float deltaDistance;
  float deltaDistanceSideways;

  // Accumulates the distance traveled during the current movement (replaces motor encoders)
  // Check if this is above a certain amount to start parallel tasks after moving a certain distance
  float totalDistance = 0;

  
  // Position and angle to the current target point
  double targetX = 0;
  double targetY = 0;
  double targetDistance = 0; // current distance from target point
  double targetDeg = 0;



// DEVICES ///////////////////////////////////


  // motors //

  motor LFBASE(PORT1, false);
  //motor LMBASE(PORT2); // Left mid
  motor LBBASE(PORT19, false);

  motor RFBASE(PORT13, true);
  //motor RMBASE(PORT14,true); // Right mid
  motor RBBASE(PORT18, true);


  // Sensors & more //

  inertial INERTIAL(PORT14);

  triport Triport(PORT22); // Get reference for three-wire ports on brain
  encoder encoderL(Triport.A); // left tracking wheel
  encoder encoderR(Triport.C); // right tracking wheel

  triport TriportExt(PORT9); // Get reference for three wire extender
  encoder encoderS(TriportExt.A); // sideways tracking wheel. Testing to see if we need this to deal with drift


  controller controllerPrim(controllerType::primary);




// MATH FUNCTIONS /////////////////////////////////////////////


  // OLD FUNCTIONS FOR MOTOR ENCODERS
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


  // Convert between inches and ticks of tracking wheels using dimensional analysis
  double inchesToTicks(double inches)
  {
    return inches * (360 / TRACKING_CIRCUMFERENCE);
  }

  double ticksToInches(double ticks)
  {
    return ticks * (TRACKING_CIRCUMFERENCE / 360);
  }

  // Return the number clamped between two numbers
  double keepInRange(double n, double bottom, double top) {
    if (n < bottom)
      n = bottom;
    if (n > top)
      n = top;
    return n;
  }


  // Find the distance between two points
  double distanceTo(double x1, double y1, double x2, double y2)
  {
      return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
  }


  // Keep rotation within 180 degrees of the reference angle (current inertial sensor angle)
  // If an angle is over 180 degrees away, this makes the bot turn the other direction because it is faster
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

    return a; // Return the new closest angle
  }




// SENSOR FUNCTIONS ///////////////////////////////////////////////////////

  // Don't really need motor encoder functions as tracking wheels can do it more accurately
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


  // Return the total distance accumulated since the start of the movement. Used to schedule parallel tasks.
  // If the bot is moving 10 in but you want something to happen when at 5 in, check if this value reaches 5 in the separate task
  double getTotalDistance()
  {
    return totalDistance;
  }

  // Clear the total distance accumulated in the beginning of each movement
  void resetTotalDistance()
  {
    totalDistance = 0;
  }

 // Check if position is not changing so robot doesn't get stuck trying to drive past a wall (todo)
  bool isStopped()
  {
    return deltaDistance = 0;
  }


  // Obtain the current values of the tracking wheel encoders
  double getRightReading() {
    return -ticksToInches(encoderR.rotation(deg)); // negative because right encoder is backwards
  }

  double getLeftReading() {
    return -ticksToInches(encoderL.rotation(deg));
  }

  double getSideReading() {
  return ticksToInches(encoderS.rotation(deg));
  }


  // Obtain the current inertial sensor rotation relative to the starting rotation
  double getDegrees()
  {
    return -INERTIAL.rotation(deg); // Important that this is negative
    // This makes positive angles CCW (left) and negative angles CW (right)
    // 0 degrees is on the positive x-axis
  }


  // Obtain the current inertial sensor rotation in radians
  // A full rotation is 2 PI instead of 360
  // Trig functions natively use radians
  double getRadians()
  {
    return getDegrees()*toRadians;
  }



  // Obtain the angle to any position from the robot's current position
  double getDegToPosition(double x, double y)
  {
    double relativeX = x - globalX;
    double relativeY = y - globalY;

    // atan2(y, x) gives the absolute angle between two points
    // This is the angle to turn to to get from the current point to the target point
    double degToPosition = toDegrees * atan2(relativeY, relativeX);

    // Prevent the robot from targeting a rotation over 180 degrees from its current rotation.
    // If it's more than 180 it's faster to turn the other direction
    degToPosition = angleWrap(degToPosition, getDegrees());
    
    return degToPosition;
  }



// STANDARD MOVEMENT ////////////////////////////

  // Set the speeds of different sides of the base
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
  void updatePosition()
  {
    // Get current values of encoders
    curLeft = getLeftReading();
    curRight = getRightReading();
    curSide = getSideReading();

    // Get change in encoder values
    deltaLeft = (curLeft - lastLeftPos);
    deltaRight = (curRight - lastRightPos);
    deltaSide = (curSide - lastSidePos);

    // Angle of arc of movement (different than inertial angle)
    // The center of the arc is an arbitrary distance away
    // The arc can be seen at the pilons document http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf (page 5)
    //curTheta = (curRight - curLeft) / (leftOffset + rightOffset); // in radians
    deltaTheta = (deltaRight - deltaLeft) / (leftOffset + rightOffset);

    // If not turning deltaX is the distance moved forward
    if (deltaTheta == 0)
    {
      deltaDistance = (deltaRight + deltaLeft) / 2; // average of change in both encoders
      //deltaDistanceSideways = deltaSide;
    }
    else // adjust for the offset of right encoder from center using the arc
    {
      // Get the radius from the center of the arc to the center of the bot
      double centerRadius = deltaRight/deltaTheta - rightOffset; // Add right encoder offset from center of bot to the right encoder's radius

      // Get the chord length (straight line distance between ends of the arc) using 2r * sin(theta/2)
      deltaDistance = 2 * centerRadius * (sin(deltaTheta/2));

       //Sideways motion is not necessary for tank drive bots because they can't shift sideways
      // Do the same to account for sideways motion
      /*double centerRadiusSideways = deltaSide/deltaTheta + sideOffset;
      deltaDistanceSideways = 2 * centerRadiusSideways * sin(deltaTheta/2);*/
    }
    
    // Estimate the average angle between the last and current position
    // Half of the arc angle works out to be the relative angle turned
    absoluteAngleOfMovement = lastInertialRadians + deltaTheta/2;//getRadians();//(lastInertialRadians + getRadians())/2;// + deltaTheta/2;//averageRadians = ( curInertialRadians + lastInertialRadians ) / 2;

    // Calculate the change in global position
    // Using trig, we can obtain the x and y components from the distance moved and the direction it moved in
    // It is converting polar coordinates (radius, angle) to (x, y) coordinates
    deltaX = deltaDistance * cos(absoluteAngleOfMovement); // x uses cosine
    deltaY = deltaDistance * sin(absoluteAngleOfMovement); // y uses sine

    // Accumulate the changes in position to the globals position variables
    globalX += deltaX;
    globalY += deltaY;
    
    //globalX += deltaDistanceSideways * sin(absoluteAngleOfMovement);
    //globalY += deltaDistanceSideways * cos(absoluteAngleOfMovement);

    totalDistance += deltaDistance; // Keep track of accumulated distance for scheduling parallel tasks

    // Save the current encoder values and rotation to use for the next update
    lastLeftPos = curLeft;
    lastRightPos = curRight;
    lastSidePos = curSide;
    lastInertialRadians = getRadians();
  }

  

  // Run one cycle of PID to obtain the speed to move to the target
  // Instead of having separate loops for moving forward and turning,
  // odometry movement uses these calculated PID speeds to move forward and turn simultaneously
  float odomFwdPID(double target, double maxSpeed)
  {
    // PID constants to tune how much each variable affects the final speed
    float Kp = 1; // Proportional makes the speed proportional to the error

    float Ki = 0.005; // Integral accumulates error to the speed over time
    // Integral is often used to to overcome friction at the end due to derivative

    float Kd = 0.175; // Derivative slows down the speed if it is too fast


    // Don't let the integral term have too much control
    float integralPowerLimit = 40 / Ki; // little less than half power
    float integralActiveZone = 15; // Only start accumulating integral
    float errorThreshold = 0.25; // Exit loop when error is less than this

    float speed = 0;

    fwd_error = target; // Error is the distance from the target

    if (fabs(fwd_error) > errorThreshold) // Check if error is over the acceptable threshold
    {
      // Only accumulate error if error is within active zone
      if (fabs(fwd_error) < integralActiveZone)
      {
        fwd_integral = fwd_integral + fwd_error; // Accumulate the error to the integral
      }
      else
      {
        fwd_integral = 0;
      }
      fwd_integral = keepInRange(fwd_integral, -integralPowerLimit, integralPowerLimit); // Limit the integral

      fwd_derivative = fwd_error - fwd_lastError; // Derivative is the change in error since the last PID cycle

      speed = (Kp * fwd_error) + (Ki * fwd_integral) + (Kd * fwd_derivative); // Multiply each variable by its tuning constant
      speed =  keepInRange(speed, -maxSpeed, maxSpeed); // Restrict the absolute value of speed to the maximum allowed value

      Brain.Screen.printAt(210, 140, "P: %.1f, I: %.1f, D: %.1f    ", (Kp * fwd_error) + (Ki * fwd_integral) + (Kd * fwd_derivative));
    }
    else
    {
      // If error threshold has been reached, set the speed to 0
      fwd_integral = 0;
      fwd_derivative = 0;
      speed = 0;
      Brain.Screen.printAt(210, 140, "PID Fwd Completed             ");
    }
    fwd_lastError = fwd_error; // Keep track of the error since last cycle
    
    return speed; // Return the final speed value
  }


  // The target angle is relative to the starting angle of the robot in degrees
  float odomTurnPID(double target, double maxSpeed)
  {
    float Kp = 0.4;
    float Ki = 0.01;
    float Kd = 0.45;
    float integralPowerLimit =
        40 / Ki;                   // little less than half power
    float integralActiveZone = 15; // degrees to start accumulating to integral
    float errorThreshold = 0.5; // Exit loop when error is less than this

    float speed = 0;


    turn_error = target - getDegrees(); // The error is the relative angle to the target angle

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

  
  // Once the majority of the turning is accounted for,
  // maintain the current angle with smoother and less drastically tuned PID
  float odomDriveStraightPID(double target, double maxSpeed)
  {
    float Kp = 0.3;
    float Ki = 0.0;
    float Kd = 0.3;
    float integralPowerLimit =
        40 / Ki;                   // little less than half power in pct
    float integralActiveZone = 15; // degrees
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



  // Turn to face a global point on the field from the robot's current location
  void turnTo(double x, double y, double turnSpeed)
  {
    double finalTurnSpeed = 1;

    while (finalTurnSpeed != 0)
    { 
      targetX = x;
      targetY = y;
      targetDeg = getDegToPosition(targetX, targetY); // Obtain the closest angle to the target position
      finalTurnSpeed = odomTurnPID(targetDeg, turnSpeed);
      
       // Turn in place towards the position
      leftDrive(-finalTurnSpeed);
      rightDrive(finalTurnSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);

      task::sleep(5);
    }
    
    Brain.Screen.printAt(210, 140, "turnTo() done.                    ");
  }


  

  // Turn to and move to the global target point simultaneously. Limit the maximum speeds of the forward and turning.
  // Modifying the ratio of turn and forward speed can change the final angle of the bot when it reaches the target
  void moveTo(double x, double y, double maxFwdSpeed, double maxTurnSpeed)
  {
    resetTotalDistance();

    // Current components of final speed to be summed up
    double curFwdSpeed = 1;
    double curTurnSpeed = 1;

    // Update the global target variables
    targetX = x;
    targetY = y;
    targetDistance = distanceTo(targetX, targetY, globalX, globalY);
    
    bool followPosition = true;
    double lastDistToGo = 0; // Keeps track of the last couple inches to finish off the movement

    // Run while forward pid is active
    // When PID is within its acceptable error threshold the speed it returns is 0
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
        targetDeg = getDegToPosition(targetX, targetY);
        
        curFwdSpeed = odomFwdPID(targetDistance, maxFwdSpeed); // calculate pid forward speed

        // When needing to turn a lot, use turn pid
        if (fabs( targetDeg - getDegrees() ) > 3)
        {
          curTurnSpeed = odomTurnPID(targetDeg, maxTurnSpeed);
        }
        else  // when maintaining current angle, use driveStraight PID
        {
          curTurnSpeed = odomDriveStraightPID(targetDeg, maxTurnSpeed); // limit max speed here
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
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);
      task::sleep(5);
    }

    Brain.Screen.printAt(210, 120, "moveTo() done.                    ");
  }



  // Turn and move backwards simultaneously to the target global position.
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

        targetDeg = getDegToPosition(targetX, targetY);
        targetDeg = angleWrap(targetDeg - 180, getDegrees()); // angle is 180 degrees so it faces backwards

        curFwdSpeed = odomFwdPID(targetDistance, maxFwdSpeed); // calculate pid forward speed

        // When needing to turn a lot, use turn pid
        if (fabs( targetDeg - getDegrees() ) > 3)
        {
          curTurnSpeed = odomTurnPID(targetDeg, maxTurnSpeed);
        }
        else  // when maintaining current angle, use driveStraight PID
        {
          curTurnSpeed = odomDriveStraightPID(targetDeg, maxTurnSpeed); // limit max speed here
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
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);
      task::sleep(5);
    }

    Brain.Screen.printAt(210, 120, "moveTo() done.                    ");
  }
  

  // Waypoints are a point the robot follows but doesn't stop at
  // Waypoints allow more control over the path of the robot
  void waypoint(double x, double y, double maxFwdSpeed, double maxTurnSpeed)
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
      // The relative direction to the point has to be the opposite sign as it was initially
      passedX = (initialX > x && globalX < x) || (initialX < x && globalX > x);
      passedY = (initialY > y && globalY < y) || (initialY < y && globalY > y);

      targetDistance = distanceTo(targetX, targetY, globalX, globalY); // Get the distance to the target

      // only turn when far enough away from the target (farther than a couple of inches)
      if (fabs(targetDistance) > 2)
      {
        targetDeg = getDegToPosition(targetX, targetY);
        
        // When needing to turn a lot, use turn pid
        if (fabs( targetDeg - getDegrees() ) > 3)
        {
          curTurnSpeed = odomTurnPID(targetDeg, maxTurnSpeed);
        }
        else  // when maintaining current angle, use driveStraight PID
        {
          curTurnSpeed = odomDriveStraightPID(targetDeg, maxTurnSpeed); // limit max speed here
        }
      }
      else // stop turning when close to the target so the bot doesn't spin around trying to correct its position
      {
        curTurnSpeed = 0;
      }

      leftDrive(maxFwdSpeed - curTurnSpeed);
      rightDrive(maxFwdSpeed + curTurnSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);
      task::sleep(5);
    }

    Brain.Screen.printAt(210, 120, "moveToWaypoint() done.                    ");
  }



  // Waypoints are a point the robot follows but doesn't stop at
  // Waypoints allow more control over the path of the robot
  void waypointRev(double x, double y, double maxFwdSpeed, double maxTurnSpeed)
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
      // The relative direction to the point has to be the opposite sign as it was initially
      passedX = (initialX > x && globalX < x) || (initialX < x && globalX > x);
      passedY = (initialY > y && globalY < y) || (initialY < y && globalY > y);

      targetDistance = -distanceTo(targetX, targetY, globalX, globalY); // Get the distance to the target

      // only turn when far enough away from the target (farther than a couple of inches)
      if (fabs(targetDistance) > 2)
      {
        targetDeg = getDegToPosition(targetX, targetY);
        targetDeg = angleWrap(targetDeg - 180, getDegrees()); // angle is 180 degrees so it faces backwards
        
        // When needing to turn a lot, use turn pid
        if (fabs( targetDeg - getDegrees() ) > 3)
        {
          curTurnSpeed = odomTurnPID(targetDeg, maxTurnSpeed);
        }
        else  // when maintaining current angle, use driveStraight PID
        {
          curTurnSpeed = odomDriveStraightPID(targetDeg, maxTurnSpeed); // limit max speed here
        }
      }
      else // stop turning when close to the target so the bot doesn't spin around trying to correct its position
      {
        curTurnSpeed = 0;
      }

      leftDrive(-maxFwdSpeed - curTurnSpeed);
      rightDrive(-maxFwdSpeed + curTurnSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);
      task::sleep(5);
    }

    Brain.Screen.printAt(210, 120, "moveToWaypoint() done.                    ");
  }


// DISPLAY ///////////////////////////


  // screen is 480 x 272
  // Draw a point on the display to represent a global position
  void drawPoint(double x, double y)
  {
    double tile_size = 30;
    double draw_pos_x = ((x / 24) * tile_size) + 105;
    double draw_pos_y = ((-y / 24) * tile_size) + 105;
    
    Brain.Screen.setPenWidth(0);
    Brain.Screen.drawCircle(draw_pos_x, draw_pos_y, 4);

    Brain.Screen.setFillColor(color(10, 80, 30)); // green in rgb
    Brain.Screen.setPenColor(white);
  }


  // Convert where the screen is tapped to global coordinates
  double screenToGlobalX(double screenX)
  {
    double tile_size = 30;
    return ((screenX - 105) / tile_size) * 24;;
  }

  // Convert where the screen is tapped to global coordinates
  double screenToGlobalY(double screenY)
  {
    double tile_size = 30;

    return -((screenY - 105) / tile_size) * 24;
  }


  // Draw the dashboard to visually display the robot's position and rotation
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

    // Line indicates the rotation found by the inertial sensor
    Brain.Screen.drawLine(draw_pos_x, // staring point of line is the bot's position
                          draw_pos_y,
                          draw_pos_x + cos(getRadians()) * 20, // End point of line calculated with polar coordinates
                          draw_pos_y - sin(getRadians()) * 20); // make y negative because down is positive on the screen
                          
    Brain.Screen.setFillColor(purple);
    drawPoint(targetX, targetY); // Draw the target point on the screen
  }



void debug()
{

  Brain.Screen.setFillColor(color(10, 80, 30)); // Set background to green in rgb
  Brain.Screen.setPenColor(white); // Set text color to white

  // Display debug values such as position, rotation, encoder values, total distancel, etc.
  Brain.Screen.printAt(210, 30, "Pos: (%.1f, %.1f)     ", globalX, globalY);
  Brain.Screen.printAt(210, 50, "Rot: %.1f deg      ", getDegrees());
  Brain.Screen.printAt(210, 70, "Enc: L:%.1f R:%.1f S:%.1f    ", getLeftReading(), getRightReading(), getSideReading());
  Brain.Screen.printAt(210, 90, "Dis: %.1f", getTotalDistance());
  
  /*
  Brain.Screen.printAt(210, 110, "lir: %.1f  aaom: %.1f", lastInertialRadians * toDegrees, absoluteAngleOfMovement * toDegrees);
  Brain.Screen.printAt(210, 130, "dth: %.1f  dd:%.1f", deltaTheta * toDegrees, deltaDistance);
  Brain.Screen.printAt(210, 150, "dl:%.1f  dr:%.1f", deltaLeft, deltaRight);
  Brain.Screen.printAt(210, 170, "dx:%.1f  dy:%.1f", deltaX, deltaY);*/
}

// Run these functions in parallel to the main autonomous/driver thread
int backgroundTasks()
{
  while (true)
  {
    updatePosition(); // Update the odometry position
    // Draw the debug values and the field dashboard
    debug();
    draw();
    task::sleep(10); // Wait some time between odometry cycles. Test making it shorter for better position estimates
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


  // Wait until the inertial sensor finishes callibrating to display the rest of the screen
  while (INERTIAL.isCalibrating())
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(210, 50, "Rot: CALLIBRATING");
  }
  Brain.Screen.setPenColor(white);

  task a(backgroundTasks); // Initiate the background tasks



  //field testing

  /*moveToWaypoint(48, 24, 50, 10);
  moveTo(0, 48, 50, 20);
  moveToRev(0, 0, 50, 20);*/

  /*moveToWaypoint(30, -20, 40, 25);
  moveTo(0, -40, 40, 25);
  moveTo(0, 0, 40, 25);*/

  
  // home testing
  /*moveToWaypoint(10, 10, 30, 40);
  moveTo(20, 20, 30, 40);*/

  // final angles: 40.3, 39.3, 39.3, 37.4, 39.1 ,38.8, 36.7
  // may need to turn to a specific angle or point afterwards to aim
  /*task::sleep(500);
  turnTo(30, 30, 22);*/


  // home testing circuit

  for (int i = 0; i < 2; i++)
  {
    // Todo: tune PID speed higher so maximum values can alter the speed
    // Maybe have a multiplier for speed instead of pct
    moveTo(25, 0, 50, 15);
    moveTo(35, 40, 45, 20);
    waypointRev(20, 20, 40, 10);
    moveToRev(0, 0, 50, 10);
  }





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

    // If a point on the screen is pressed, move to the global position of that point for debug purposes
    if (Brain.Screen.pressing())
    {
      targetX = screenToGlobalX(Brain.Screen.xPosition());
      targetY = screenToGlobalY(Brain.Screen.yPosition());
      //turnTo(targetX, targetY, 20);
      //moveTo(targetX, targetY, 20, 25);
      moveTo(0, 0, 20, 25);

      /*updatePosition();
      task::sleep(500);*/
    }
    
    // control the robot more precisely than with joysticks
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
      // Control with joysticks if buttons are not used. Joystick value defaults to 0 when not touched so the speed will be 0.
      leftDrive(controllerPrim.Axis3.value());
      rightDrive(controllerPrim.Axis2.value());
      //stopBase();
    }

    task::sleep(5);
  }
}
