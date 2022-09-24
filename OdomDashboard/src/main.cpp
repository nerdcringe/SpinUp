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

  // CONSTANTS //

  #define PI 3.14159265
  const double toRadians = PI / 180.0; // multiply degrees by this to convert to radians
  const double toDegrees = 180.0 / PI; // multiply radians by this to convert to degrees

  //const double WHEEL_CIRCUMFERENCE = 3.25 * PI; // Circumference of powered wheels (diameter * PI)
  const double TRACKING_CIRCUMFERENCE = 2.75 * PI; // Circumference of tracking wheels (diameter * PI)

  // tracking wheel perpendicular distances from true center of robot.
  // Seen at page 4 of Pilons odometry doc (http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)
  const double leftOffset =  4.875; // left
  const double rightOffset = 4.875; // right
  const double sideOffset = 0.25; // sideways



  // PID VARIABLES //

  /*
    PID is a way to move some distance or turn to some angle.
    The distance or angle from a target is called error.

    To reduce error precisely, PID uses three variables

      P: Proportional
        - Fast when error is high and slow when eror is low
        - Majority of the speed comes from P term
      
      I: Integral
        - Accumulates if far away from the target for a while
        - When slowing down at the end from P, I makes sure speed is fast enough to overcome friction & weight of robot

      D: Derivative
        - If position is changing too fast, smooth out those changes
        - Dampens any unwanted oscillations while trying to settle towards the target

  */

  float fwd_error = 0; // Distance from target forward distance
  float fwd_lastError = 0; // Keep track of last error for the derivative (rate of change)
  float fwd_integral = 0; // Integral accumulates the error, speeding up if target is not reached fast enough
  float fwd_derivative = 0; // Derivative smooths out oscillations and counters sudden changes

  float turn_error = 0; // Relative angle from the target rotation (degrees)
  float turn_lastError = 0;
  float turn_integral = 0;
  float turn_derivative = 0;


  // ODOMETRY VARIABLES //

  // *** Global position on the field ***
  // These values always start at (0, 0) so everything is relative to the starting location of the robot
  double globalX = 0;
  double globalY = 0;

  // Current positions of tracking wheel encoders
  float curLeft = 0;
  float curRight = 0;
  float curSide = 0;

  // Positions of tracking wheel encoders from last update
  float lastLeftPos = 0;
  float lastRightPos = 0;
  float lastSidePos = 0;
  double lastInertialRadians = 0; // Angle (radians) returned by inertial sensor from last update

  // Change in tracking wheel encoder positions since last update
  float deltaLeft = 0;
  float deltaRight = 0;
  float deltaSide = 0;

  // Change in arc theta (twice the change in inertial radians) since last update
  float deltaTheta = 0;

  // Change in global position since last update
  double deltaX = 0;
  double deltaY = 0;

  // Distance moved in the bot's direction
  float deltaDistance;
  float deltaDistanceSideways;
  // The direction of the movement tracked by odometry, in radians
  double absoluteAngleOfMovement = 0;


  // Accumulates the distance traveled during the current movement (replaces motor encoders)
  // Check if this is above a certain amount to start parallel tasks after moving a certain distance
  float totalDistance = 0;


  // Position and angle to the current target point
  double targetX = 0;
  double targetY = 0;
  double targetDeg = 0;

  double targetDistance = 0; // Current straightest path distance to target point


  //bool exitLoop = false; // Exit a movement loop if robot is stuck and encoders don't detect movement

// DEVICES ///////////////////////////////////


  // motors //

  motor LBBASE(PORT17, false);
  //motor LMBASE(PORT2); // Left mid
  motor LFBASE(PORT19, false);

  motor RBBASE(PORT13, true);
  //motor RMBASE(PORT14,true); // Right mid
  motor RFBASE(PORT18, true);


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
  double distanceTo(double x1, double y1)
  {
      return sqrt(pow(globalX - x1, 2) + pow(globalY - y1, 2) * 1.0);
  }



// SENSOR FUNCTIONS ///////////////////////////////////////////////////////

  // Old motor encoder distance tracking
  /*
  double getTotalDistance() // return average motor encoder value
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

  void resetTotalDistance()
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


  int stopTime = 0;
 // Check if position is not changing so robot doesn't get stuck trying to drive past a wall (todo)
  bool isStopped()
  {
    if (fabs(deltaRight) < 0.001)
    {
      stopTime += 1;
    }
    else
    {
      stopTime = 0;
    }
    if (stopTime > 50)
    {
      stopTime = 0;
      return true;
    }
    return false;
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



  // Keep rotation within 180 degrees of the reference angle (current inertial sensor angle)
  // If an angle is over 180 degrees away, this makes the bot turn the other direction because it is faster
  double angleWrap(double a)
  {
    double robotAngle = getDegrees();
    while(a > robotAngle + 180) // Subtract 360 if angle is too large
    {
      a -= 360;
    }
    while(a <= robotAngle - 180) // Add 360 if angle is too large
    {
      a += 360;
    }

    return a; // Return the new closest angle
  }


  // Obtain the angle to any position from the robot's current position
  double getDegToPosition(double x, double y)
  {
    double relativeX = x - globalX;
    double relativeY = y - globalY;

    // atan2(y, x) gives the absolute angle from the origin to the specified point
    // This is the angle to turn to to get from the current point to the target point
    double degToPosition = toDegrees * atan2(relativeY, relativeX);

    // Prevent the robot from targeting a rotation over 180 degrees from its current rotation.
    // If it's more than 180 it's faster to turn the other direction
    degToPosition = angleWrap(degToPosition);
    
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


  // Run one cycle of PID to obtain the speed to move to the target
  // Instead of having separate loops for moving forward and turning,
  // odometry movement uses these calculated PID speeds to move forward and turn simultaneously
  float fwdPIDCycle(double targetDist, double maxSpeed)
  { 
    float Kp = 5; // Proportional makes the speed proportional to the error

    float Ki = 0.01; // Integral accumulates error to the speed over time
    // Integral is often used to to overcome friction at the end due to derivative

    float Kd = 1;//45.0; // Derivative slows down the speed if it is too fast


    // Don't let the integral term have too much control
    float integralPowerLimit = 40 / Ki; // little less than half power
    float integralActiveZone = 10; // Only start accumulating integral
    float errorThreshold = 0.25; // Exit loop when error is less than this

    float speed = 0;

    fwd_error = targetDist; // Error is the distance from the target

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

      if (speed > 0 && speed < 2) {
        speed = 2;
      } if (speed < 0 && speed > -2) {
        speed = -2;
      }

      Brain.Screen.printAt(210, 140, "P: %.1f, I: %.1f, D: %.1f    ", (Kp * fwd_error), (Ki * fwd_integral), (Kd * fwd_derivative));
    //Brain.Screen.printAt(210, 120, "%f  ", fwd_error - fwd_lastError);
    }
    else
    {
      // If error threshold has been reached, set the speed to 0
      fwd_integral = 0;
      fwd_derivative = 0;
      speed = 0;
      //Brain.Screen.printAt(210, 140, "PID Fwd Completed             ");
    }
    fwd_lastError = fwd_error; // Keep track of the error since last cycle
    
    return speed; // Return the final speed value
  }


  // The target angle is relative to the starting angle of the robot in degrees
  float turnPIDCycle(double targetDegree, double maxSpeed)
  {
    float Kp = 0.56;
    float Ki = 0.00075;//0.01;
    float Kd = 0.5;//1;
    float integralPowerLimit =
        40 / Ki;                   // little less than half power
    float integralActiveZone = 15; // degrees to start accumulating to integral
    float errorThreshold = 0.5; // Exit loop when error is less than this

    float speed = 0;

    turn_error = targetDegree - getDegrees(); // The error is the relative angle to the target angle

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
      //Brain.Screen.printAt(210, 160, "PID Turn Completed             ");
    }
    turn_lastError = turn_error;

    return speed;
  }


  // Run a closed PID loop for liinear distance in inches
  void forwardPID(double targetInches, double maxSpeed, int timeoutMillis = -1) // default timeout is -1, meaning unlimited
  {
    double curentSpeed = 1;

    timer Timer;
    Timer.reset(); // start timer for timeout

    resetTotalDistance();
    // While time isn't up or time isn't set (default value of -1)
    while (curentSpeed != 0 && (Timer.time() < timeoutMillis || timeoutMillis == -1))
    {
      double currentDist = targetInches - getTotalDistance(); // calculate the distance from target

      curentSpeed = fwdPIDCycle(currentDist, maxSpeed); // plug in distance and speed into PID
      leftDrive(curentSpeed);
      rightDrive(curentSpeed);

      task::sleep(10);
    }
    stopBase();
  }

  
  // Run a closed PID loop for turning to a specific absolute degree
  // Setting targetDeg to 0 degrees always returns to the starting angle
  void turnPID(double targetDeg, double maxSpeed, int timeoutMillis=-1) // default timeout is -1, meaning unlimited
  {
    double curentSpeed = 1;

    timer Timer;
    Timer.reset(); // start timer for timeout

    resetTotalDistance();
    // While time isn't up or time isn't set (default value of -1)
    while (curentSpeed != 0 && (Timer.time() < timeoutMillis || timeoutMillis == -1))
    {
      curentSpeed = turnPIDCycle(targetDeg, maxSpeed);
      leftDrive(-curentSpeed);
      rightDrive(curentSpeed);

      task::sleep(10);
    }
    stopBase();
  }



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
    deltaTheta = (deltaRight - deltaLeft) / (leftOffset + rightOffset);

    // If not turning deltaX is the distance moved forward
    if (deltaTheta == 0)
    {
      deltaDistance = (deltaRight + deltaLeft) / 2; // average of change in both encoders
    }
    else // adjust for the offset of right encoder from center using the arc
    {
      // Get the radius from the center of the arc to the center of the bot
      double centerRadius = deltaRight/deltaTheta - rightOffset; // Add right encoder offset from center of bot to the right encoder's radius

      // Get the chord length (straight line distance between ends of the arc) using 2r * sin(theta/2)
      deltaDistance = 2 * centerRadius * (sin(deltaTheta/2));
    }
    
    // Estimate the average angle between the last and current position
    // Half of the arc angle works out to be the relative angle turned
    absoluteAngleOfMovement = lastInertialRadians + deltaTheta/2;//getRadians();//(lastInertialRadians + getRadians())/2;// + deltaTheta/2;//averageRadians = ( curInertialRadians + lastInertialRadians ) / 2;

    // Calculate the change in global position
    // Using trig, we can obtain the x and y components from the distance moved and the direction it moved in
    // It is converting polar coordinates (radius, angle) to (x, y) coordinates
    deltaX = deltaDistance * cos(absoluteAngleOfMovement); // + deltaDistanceSideways * sin(absoluteAngleOfMovement);
    deltaY = deltaDistance * sin(absoluteAngleOfMovement); // + deltaDistanceSideways * cos(absoluteAngleOfMovement);

    // Accumulate the changes in position to the globals position variables
    globalX += deltaX;
    globalY += deltaY;
    
    totalDistance += deltaDistance; // Keep track of accumulated distance for scheduling parallel tasks

    // Save the current encoder values and rotation to use for the next update
    lastLeftPos = curLeft;
    lastRightPos = curRight;
    lastSidePos = curSide;
    lastInertialRadians = getRadians();
  }
  

  
  // Set the current target position that will be referenced by later movement functions
  void setTarget(double x, double y)
  {
    targetX = x;
    targetY = y;
  }
  

  // Turn to face the target position, regardless of the robot's current position
  void turnToTarget(double maxTurnSpeed)
  {
    double finalTurnSpeed = 1;

    while (finalTurnSpeed != 0)
    {
      targetDeg = getDegToPosition(targetX, targetY); // Obtain the closest angle to the target position

      finalTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed); // Plug angle into turning PID and get the resultant speed
      
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
  void moveToTarget(double maxFwdSpeed, double maxTurnSpeed)
  {
    resetTotalDistance();
    stopTime = 0;

    // Current components of final speed to be summed up
    double curFwdSpeed = 1;
    double curTurnSpeed = 1;

    // Update the global target variables
    targetDistance = distanceTo(targetX, targetY);

    // Run while forward pid is active
    // When PID is within its acceptable error threshold it returns 0 speed
    // Only turn when farther than a few inches
    while (curFwdSpeed != 0 && fabs(targetDistance) > 3/* && !isStopped()*/)
    {
      targetDistance = distanceTo(targetX, targetY);
      targetDeg = getDegToPosition(targetX, targetY);
      
      curFwdSpeed = fwdPIDCycle(targetDistance, maxFwdSpeed); // calculate pid forward speed
      curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed);

      
      leftDrive(curFwdSpeed - curTurnSpeed);
      rightDrive(curFwdSpeed + curTurnSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);
      task::sleep(5);
    }

    // When within a few inches of the point, just move foward without turning
    // so the bot doesn't run in circles around the point if it overshoots by a hair
    double pureFwdDistance = getTotalDistance() + targetDistance; // add current total distance to distance to target

    while (curFwdSpeed != 0/* && !isStopped()*/)
    {
      curFwdSpeed = fwdPIDCycle(pureFwdDistance - getTotalDistance(), maxFwdSpeed); // error = desired - actual distance
      curTurnSpeed = 0;
      
      leftDrive(curFwdSpeed);
      rightDrive(curFwdSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);
      task::sleep(5);
    }
    stopBase();

    Brain.Screen.printAt(210, 120, "moveTo() done.                    ");
  }




  // Turn to and move to the global target point simultaneously. Limit the maximum speeds of the forward and turning.
  // Modifying the ratio of turn and forward speed can change the final angle of the bot when it reaches the target
  void moveToTargetRev(double maxFwdSpeed, double maxTurnSpeed)
  {
    resetTotalDistance();
    stopTime = 0;

    // Current components of final speed to be summed up
    double curFwdSpeed = 1;
    double curTurnSpeed = 1;

    // Update the global target variables
    targetDistance = -distanceTo(targetX, targetY);
    
    // Run while forward pid is active
    // When PID is within its acceptable error threshold it returns 0 speed
    // Only turn when farther than a few inches
    while (curFwdSpeed != 0 && fabs(targetDistance) > 3/* && !isStopped()*/)
    {
      targetDistance = -distanceTo(targetX, targetY); // negative
      targetDeg = getDegToPosition(targetX, targetY);
      targetDeg = angleWrap(targetDeg - 180); // angle is 180 degrees so it faces backwards

      // run one cycle of forwad and turn pid
      curFwdSpeed = fwdPIDCycle(targetDistance, maxFwdSpeed);
      curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed);

      leftDrive(curFwdSpeed - curTurnSpeed);
      rightDrive(curFwdSpeed + curTurnSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);
      task::sleep(5);
    }

    // When within a few inches of the point, just move foward without turning
    // so the bot doesn't run in circles around the point if it overshoots by a hair
    double pureFwdDistance = getTotalDistance() + targetDistance; // add current total distance to distance to target

    while (curFwdSpeed != 0/* && !isStopped()*/)
    {
      curFwdSpeed = fwdPIDCycle(pureFwdDistance - getTotalDistance(), maxFwdSpeed); // error = desired - actual distance
      curTurnSpeed = 0;
      
      leftDrive(curFwdSpeed);
      rightDrive(curFwdSpeed);
      Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);
      task::sleep(5);
    }
    stopBase();

    Brain.Screen.printAt(210, 120, "moveToTarget() done.                    ");
  }



  // Pass the point but don't stop at it
  // This allows more control over the path of the robot by setting interrmediate locations
  void passTarget(double maxFwdSpeed, double maxTurnSpeed)
  {
    resetTotalDistance();

    // Current components of final speed to be summed up
    double curTurnSpeed = 1;

    double initialX = globalX;
    double initialY = globalY;

    targetDistance = distanceTo(targetX, targetY);
    
    bool passedX = false;
    bool passedY = false;
    
    // run while the robot has not passed the point's x and y position yet
    while (!passedX || !passedY)
    {
      // Check if the point has been passed yet
      // The relative direction to the point has to be the opposite sign as it was initially
      passedX = (initialX >= targetX && globalX <= targetX) || (initialX <= targetX && globalX >= targetX);
      passedY = (initialY >= targetY && globalY <= targetY) || (initialY <= targetY && globalY >= targetY);

      targetDistance = distanceTo(targetX, targetY); // Get the distance to the target

      // only turn when far enough away from the target (farther than a couple of inches)
      if (fabs(targetDistance) > 2)
      {
        targetDeg = getDegToPosition(targetX, targetY);
        curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed); // Run one cycle of turning PID
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

    Brain.Screen.printAt(210, 120, "passTarget() done.                    ");
  }


  // Pass the target in reverse
  void passTargetRev(double maxFwdSpeed, double maxTurnSpeed)
  {
    resetTotalDistance();

    // Current components of final speed to be summed up
    double curTurnSpeed = 1;

    double initialX = globalX;
    double initialY = globalY;

    targetDistance = distanceTo(targetX, targetY);
    
    bool passedX = false;
    bool passedY = false;
    
    // run while the robot has not passed the point's x and y position yet
    while (!passedX || !passedY)
    {
      // Check if the point has been passed yet
      // The relative direction to the point has to be the opposite sign as it was initially
      passedX = (initialX >= targetX && globalX <= targetX) || (initialX <= targetX && globalX >= targetX);
      passedY = (initialY >= targetY && globalY <= targetY) || (initialY <= targetY && globalY >= targetY);

      targetDistance = -distanceTo(targetX, targetY); // Get the distance to the target

      // only turn when far enough away from the target (farther than a couple of inches)
      if (fabs(targetDistance) > 2)
      {
        targetDeg = getDegToPosition(targetX, targetY);
        targetDeg = angleWrap(targetDeg - 180); // angle is 180 degrees so it faces backwards
        
        curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed);
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

    Brain.Screen.printAt(210, 120, "passTargetRev() done.                    ");
  }




// DISPLAY ///////////////////////////


  const double tileDrawSize = 30; // tiles on screen have 30 pixel long sides

  // screen is 480 x 272
  // Draw a point on the display to represent a global position
  void drawPoint(double x, double y)
  {
    double draw_pos_x = ((x / 24) * tileDrawSize) + 105;
    double draw_pos_y = ((-y / 24) * tileDrawSize) + 105;
    
    Brain.Screen.setPenWidth(0);
    Brain.Screen.drawCircle(draw_pos_x, draw_pos_y, 4);

    Brain.Screen.setFillColor(color(10, 80, 30)); // green in rgb
    Brain.Screen.setPenColor(white);
  }


  // Convert where the screen is tapped to global coordinates
  double screenToGlobalX(double screenX)
  {
    return ((screenX - 105) / tileDrawSize) * 24;
  }

  // Convert where the screen is tapped to global coordinates
  double screenToGlobalY(double screenY)
  {
    return -((screenY - 105) / tileDrawSize) * 24;
  }


  // Draw the dashboard to visually display the robot's position and rotation
  void draw()
  {
    // Draw grid of the field
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(color(100, 100, 100));
    Brain.Screen.setPenWidth(2);

    for (int x = 0; x < 6; x++)
    {
      for (int y = 0; y < 6; y++)
      {
        Brain.Screen.drawRectangle(x * tileDrawSize + 15, y * tileDrawSize + 15, tileDrawSize, tileDrawSize); // fill entire screen
      }
    }
    Brain.Screen.printAt(170, 100, "+x");
    Brain.Screen.printAt(20, 100, "-x");
    Brain.Screen.printAt(110, 30, "+y");
    Brain.Screen.printAt(110, 190, "-y");
    
    // Draw robot
    Brain.Screen.setFillColor(white);
    Brain.Screen.setPenWidth(0);
    double draw_pos_x = ((globalX/24) * tileDrawSize) + 105;
    double draw_pos_y = ((-globalY/24) * tileDrawSize) + 105; // make y negative because down is positive on the screen

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



  float lastGraphX = 0;
  float lastGraphY = 0;
  // Display the graph of a variable over time
  void graph()
  {
    float x = timer::system()/35;
    float y = totalDistance * 50; //LFBASE.velocity(rpm);

    // draw y = 0 line
    Brain.Screen.setPenWidth(1);
    Brain.Screen.setPenColor(color(255, 255, 255));
    Brain.Screen.setFillColor(color(255, 255, 255));
    Brain.Screen.drawLine(0, 136, 480, 136);

    // Draw data
    Brain.Screen.setPenWidth(1);
    Brain.Screen.setPenColor(color(255, 150, 0));
    Brain.Screen.setFillColor(color(255, 150, 0));
    Brain.Screen.drawLine(lastGraphX, -lastGraphY + 136, x, -y + 136);
    
    lastGraphX = x;
    lastGraphY = y;
  }



void values()
{

  Brain.Screen.setFillColor(color(10, 80, 30)); // Set background to green in rgb
  Brain.Screen.setPenColor(white); // Set text color to white

  // Display debug values such as position, rotation, encoder values, total distancel, etc.
  Brain.Screen.printAt(210, 30, "Pos: (%.1f, %.1f)     ", globalX, globalY);
  Brain.Screen.printAt(210, 50, "Rot: %.1f deg      ", getDegrees());
  Brain.Screen.printAt(210, 70, "Enc: L:%.1f R:%.1f S:%.1f    ", getLeftReading(), getRightReading(), getSideReading());
  Brain.Screen.printAt(210, 90, "Dis: %.7f", getTotalDistance());

  /* Brain.Screen.printAt(210, 110, "Stoptime: %d   ", stopTime);
  Brain.Screen.printAt(210, 130, "isStopped: %d   ", isStopped());*/

  /* Brain.Screen.printAt(210, 110, "lir: %.1f  aaom: %.1f", lastInertialRadians * toDegrees, absoluteAngleOfMovement * toDegrees);
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

    values();
    draw();
    //graph();
    task::sleep(10); // Wait some time between odometry cycles. Test making it shorter for better position estimates
  }
  return 0;
}

int test()
{
  while (getTotalDistance() < 10)
  {
    task::sleep(10);
  } 
  Brain.Screen.setFillColor(color(200, 80, 30)); // green in rgb
  Brain.Screen.setPenWidth(0);
  Brain.Screen.drawRectangle(0, 0, 480, 272); // fill entire screen

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

  //turnPID(45,20);
  //turnPID(-180,100);
  //forwardPID(12, 50);
  //forwardPID(-15, 35);

  /*setTarget(25, 0);
  moveToTarget(35, 5);
  
  setTarget(25, 25);
  moveToTarget(20, 25);*/
/*
  setTarget(0, 25);
  moveToTarget(20, 25);

  setTarget(0, 0);
  moveToTarget(20, 25);*/

/*
setTarget(-25,0);
//moveToTarget(45,5);
turnToTarget(25);
*/
/*
setTarget(12,0);//-10
passTarget(30,14);

setTarget(23,-15);//-20
passTarget(30,20);

setTarget(9,-24);//-20
passTarget(30,20);

setTarget(2,-36);//-20
passTarget(25,30);

setTarget(13,-44);//-20
moveToTarget(24,20);
*/
  //field testing

  /*moveToWaypoint(48, 24, 50, 10);
  moveTo(0, 48, 50, 20);
  moveToRev(0, 0, 50, 20);*/

  /*moveToWaypoint(30, -20, 40, 25);
  moveTo(0, -40, 40, 25);
  moveTo(0, 0, 40, 25);*/



  while (1)
  {

    // If a point on the screen is pressed, move to the global position of that point for debug purposes
    if (Brain.Screen.pressing())
    {
      targetX = screenToGlobalX(Brain.Screen.xPosition());
      targetY = screenToGlobalY(Brain.Screen.yPosition());
      turnToTarget(30);
      //moveToTarget(25, 25);
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

      stopBase();
    }


    task::sleep(5);
  }
}
