#include "vex.h"

// tracking wheel perpendicular distances from true center of robot.
// Seen at page 4 of Pilons odometry doc (http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)
const double leftOffset =  4.875; // left
const double rightOffset = 4.875; // right


/*
  Odometry is a system that tracks xy coordinates on the field. Instead of relative commands such as
  move forward, turn right 90 degrees, etc, it can go to a specific set of (x, y) coordinates.

  Position tracking is done by using the direction of the robot and the distance it moves every ~10 milliseconds.
  It converts this to a change in x,y coordinates using trig. The changes in position are added up to get total position.
  There are some extra adjustments made because the tracking wheels aren't centered but it doesn't change too much conceptually.
*/


// *** Global position on the field ***
// These values always start at (0, 0) so everything is relative to the starting location of the robot
double globalX = 0;
double globalY = 0;

double getGlobalX() { return globalX; }
double getGlobalY() { return globalY; }


// Current positions of tracking wheel encoders
float curLeft = 0;
float curRight = 0;
float curSide = 0;

// Positions of tracking wheel encoders from last update
float lastLeftPos = 0;
float lastRightPos = 0;
double lastInertialRadians = 0; // Angle (radians) returned by inertial sensor from last update

// Change in tracking wheel encoder positions since last update
float deltaLeft = 0;
float deltaRight = 0;

float deltaTheta = 0; // Change in arc theta (twice the change in inertial radians) since last update

// Change in global position since last update
double deltaX = 0;
double deltaY = 0;

// Distance moved in the bot's direction
float deltaDistance;
// The direction of the movement tracked by odometry, in radians
double absoluteAngleOfMovement = 0;


// Position and angle to the current target point
double targetX = 0;
double targetY = 0;
double targetDistance = 0; // Current straightest path distance to target point
double targetDeg = 0;


// Find the distance between two points
double getDistanceTo(double x, double y)
{
    return sqrt(pow(globalX - x, 2) + pow(globalY - y, 2) * 1.0);
}


// Keep rotation within 180 degrees of the current robot angle
// If an angle is over 180 degrees away, make the robot turn the other way because it's faster the other way
double angleWrap(double deg)
{
  double robotAngle = getRotationDeg();
  while(deg > robotAngle + 180) // Subtract 360 if angle is too large
  {
    deg -= 360; // Equivalent angle but subtract a rotation
  }
  while(deg <= robotAngle - 180) // Add 360 if angle is too large
  {
    deg += 360; // Equivalent angle but add a rotation
  }

  return deg; // Return the new closest angle
}


// Obtain the angle to any position from the robot's current position
double getDegToPoint(double x, double y)
{
  double relativeX = x - globalX;
  double relativeY = y - globalY;

  // atan2(y, x) gives the absolute angle from the origin to the specified point
  // This is the angle to turn to to get from the current point to the target point
  double deg = toDegrees * atan2(relativeY, relativeX);

  // Prevent the robot from targeting a rotation over 180 degrees from its current rotation.
  // If it's more than 180 it's faster to turn the other direction
  deg = angleWrap(deg);
  
  return deg;
}



// Do one calculation to update global position
// Run in a parallel task during auton
void updatePosition()
{
  // Get current values of encoders
  curLeft = getLeftReading();
  curRight = getRightReading();

  // Get change in encoder values
  deltaLeft = (curLeft - lastLeftPos);
  deltaRight = (curRight - lastRightPos);

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
    double centerRadius = deltaRight/deltaTheta - rightOffset; // Add right encoder offset from center of bot to the right encoder's radi

    // Get the chord length (straight line distance between ends of the arc) using 2r * sin(theta/2)
    deltaDistance = 2 * centerRadius * (sin(deltaTheta/2));
  }
  
  // Estimate the average angle between the last and current position (half the arc's angle)
  absoluteAngleOfMovement = lastInertialRadians + deltaTheta/2;

  // Calculate the change in global position
  // Using trig, we can obtain the x and y components from the distance moved and the direction it moved in
  // It is converting polar coordinates (radius, angle) to (x, y) coordinates
  deltaX = deltaDistance * cos(absoluteAngleOfMovement); // update these for debugging purposes
  deltaY = deltaDistance * sin(absoluteAngleOfMovement);

  // Accumulate the changes in position to the globals position variables
  globalX += deltaX;
  globalY += deltaY;
  
  // Save the current encoder values and rotation to use for the next update
  lastLeftPos = curLeft;
  lastRightPos = curRight;
  lastInertialRadians = getRotationRad();
}

/*
// Update position for GPS sensor
// Use this instead of the longer updatePosition if you're using the GPS sensor
void updatePositionGPS()
{
  globalX = GPS.xPosition(distanceUnits::in);
  globalY = GPS.yPosition(distanceUnits::in);
}
*/



// AUTON STUFF ////////////////////////////////////////////////////////////////////////////////


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

  while (finalTurnSpeed != 0) // If within acceptable distance, PID output is zero.
  {
    targetDeg = getDegToPoint(targetX, targetY); // Obtain the closest angle to the target position

    finalTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed); // Plug angle into turning PID and get the resultant speed
    
      // Turn in place towards the position
    setLeftBase(-finalTurnSpeed);
    setRightBase(finalTurnSpeed);
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

  // Current components of final speed to be summed up
  double curFwdSpeed = 1;
  double curTurnSpeed = 1;

  // Update the global target variables
  targetDistance = getDistanceTo(targetX, targetY);

  // Run while forward pid is active
  // When PID is within its acceptable error threshold it returns 0 speed
  // Only turn when farther than a few inches
  while (curFwdSpeed != 0 && fabs(targetDistance) > 3/* && !isStopped()*/)
  {
    targetDistance = getDistanceTo(targetX, targetY);
    targetDeg = getDegToPoint(targetX, targetY);
    
    curFwdSpeed = fwdPIDCycle(targetDistance, maxFwdSpeed); // calculate pid forward speed
    curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed);
    
    setLeftBase(curFwdSpeed - curTurnSpeed);
    setRightBase(curFwdSpeed + curTurnSpeed);
    Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);
    task::sleep(5);
  }

  // When within a few inches of the point, just move foward without turning
  // so the bot doesn't run in circles around the point if it overshoots by a hair
  double finalDistance = getTotalDistance() + targetDistance; // add current total distance to distance to target

  while (curFwdSpeed != 0/* && !isStopped()*/)
  {
    curFwdSpeed = fwdPIDCycle(finalDistance - getTotalDistance(), maxFwdSpeed); // error = desired - actual distance
    curTurnSpeed = 0;
    
    setLeftBase(curFwdSpeed);
    setRightBase(curFwdSpeed);
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

  // Current components of final speed to be summed up
  double curFwdSpeed = 1;
  double curTurnSpeed = 1;

  // Update the global target variables
  targetDistance = -getDistanceTo(targetX, targetY);
  
  // Run while forward pid is active
  // When PID is within its acceptable error threshold it returns 0 speed
  // Only turn when farther than a few inches
  while (curFwdSpeed != 0 && fabs(targetDistance) > 3/* && !isStopped()*/)
  {
    targetDistance = -getDistanceTo(targetX, targetY); // negative
    targetDeg = getDegToPoint(targetX, targetY);
    targetDeg = angleWrap(targetDeg - 180); // angle is 180 degrees so it faces backwards

    // run one cycle of forwad and turn pid
    curFwdSpeed = fwdPIDCycle(targetDistance, maxFwdSpeed);
    curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed);

    setLeftBase(curFwdSpeed - curTurnSpeed);
    setRightBase(curFwdSpeed + curTurnSpeed);
    Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);
    task::sleep(5);
  }

  // When within a few inches of the point, just move foward without turning
  // so the bot doesn't run in circles around the point if it overshoots by a hair
  double finalDistance = getTotalDistance() + targetDistance; // add current total distance to distance to target

  while (curFwdSpeed != 0/* && !isStopped()*/)
  {
    curFwdSpeed = fwdPIDCycle(finalDistance - getTotalDistance(), maxFwdSpeed); // error = desired - actual distance
    curTurnSpeed = 0;
    
    setLeftBase(curFwdSpeed);
    setRightBase(curFwdSpeed);
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

  targetDistance = getDistanceTo(targetX, targetY);
  
  bool passedX = false;
  bool passedY = false;
  
  // run while the robot has not passed the point's x and y position yet
  while (!passedX || !passedY)
  {
    // Check if the point has been passed yet
    // The relative direction to the point has to be the opposite sign as it was initially
    passedX = (initialX >= targetX && globalX <= targetX) || (initialX <= targetX && globalX >= targetX);
    passedY = (initialY >= targetY && globalY <= targetY) || (initialY <= targetY && globalY >= targetY);

    targetDistance = getDistanceTo(targetX, targetY); // Get the distance to the target

    // only turn when far enough away from the target (farther than a couple of inches)
    if (fabs(targetDistance) > 2)
    {
      targetDeg = getDegToPoint(targetX, targetY);
      curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed); // Run one cycle of turning PID
    }
    else // stop turning when close to the target so the bot doesn't spin around trying to correct its position
    {
      curTurnSpeed = 0;
    }

    setLeftBase(maxFwdSpeed - curTurnSpeed);
    setRightBase(maxFwdSpeed + curTurnSpeed);
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

  targetDistance = getDistanceTo(targetX, targetY);
  
  bool passedX = false;
  bool passedY = false;
  
  // run while the robot has not passed the point's x and y position yet
  while (!passedX || !passedY)
  {
    // Check if the point has been passed yet
    // The relative direction to the point has to be the opposite sign as it was initially
    passedX = (initialX >= targetX && globalX <= targetX) || (initialX <= targetX && globalX >= targetX);
    passedY = (initialY >= targetY && globalY <= targetY) || (initialY <= targetY && globalY >= targetY);

    targetDistance = -getDistanceTo(targetX, targetY); // Get the distance to the target

    // only turn when far enough away from the target (farther than a couple of inches)
    if (fabs(targetDistance) > 2)
    {
      targetDeg = getDegToPoint(targetX, targetY);
      targetDeg = angleWrap(targetDeg - 180); // angle is 180 degrees so it faces backwards
      
      curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed);
    }
    else // stop turning when close to the target so the bot doesn't spin around trying to correct its position
    {
      curTurnSpeed = 0;
    }

    setLeftBase(-maxFwdSpeed - curTurnSpeed);
    setRightBase(-maxFwdSpeed + curTurnSpeed);
    Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetDeg);
    task::sleep(5);
  }

  Brain.Screen.printAt(210, 120, "passTargetRev() done.                    ");
}



// DISPLAY ///////////////////////////

const double drawSize = 30; // tiles on screen have 30 pixel long side

// Draw a point on the display to represent a global position
// screen is 480 x 272
void drawPoint(double x, double y)
{
  double draw_pos_x = ((x / 24) * drawSize) + 105;
  double draw_pos_y = ((-y / 24) * drawSize) + 105;
  
  Brain.Screen.setPenWidth(0);
  Brain.Screen.drawCircle(draw_pos_x, draw_pos_y, 4);

  Brain.Screen.setFillColor(color(10, 80, 30)); // green in rgb
  Brain.Screen.setPenColor(white);
}


// Convert where the screen is tapped to global coordinates on the field display
// Only really used for debugging
double getScreenTouchX()
{
  return ((Brain.Screen.xPosition() - 105) / drawSize) * 24;
}
double getScreenTouchY()
{
  return -((Brain.Screen.yPosition() - 105) / drawSize) * 24;
}


// Display the robot's position and rotation on the field
void odomDisplay()
{
  // Draw Y

  Brain.Screen.setPenColor(yellow);
  Brain.Screen.setPenWidth(40);
  Brain.Screen.drawLine(100, 125, 100, 220);
  Brain.Screen.drawLine(100, 130, 35, 35);
  Brain.Screen.drawLine(100, 130, 165, 35);


  /*
  // Draw grid of the field
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(color(100, 100, 100));
  Brain.Screen.setPenWidth(2);

  for (int x = 0; x < 6; x++)
  {
    for (int y = 0; y < 6; y++)
    {
      Brain.Screen.drawRectangle(x * drawSize + 15, y * drawSize + 15, drawSize, drawSize); // fill entire screen
    }
  }
  Brain.Screen.printAt(170, 100, "+x");
  Brain.Screen.printAt(20, 100, "-x");
  Brain.Screen.printAt(110, 30, "+y");
  Brain.Screen.printAt(110, 190, "-y");
  

  // Draw robot
  Brain.Screen.setFillColor(white);
  Brain.Screen.setPenWidth(0);
  double draw_pos_x = ((globalX/24) * drawSize) + 105;
  double draw_pos_y = ((-globalY/24) * drawSize) + 105; // make y negative because down is positive on the screen

  // Position of circle indicates the position found by odometry
  Brain.Screen.drawCircle(draw_pos_x, draw_pos_y, 6);
  Brain.Screen.setPenColor(red);
  Brain.Screen.setPenWidth(5);

  // Line indicates the rotation found by the inertial sensor
  Brain.Screen.drawLine(draw_pos_x, // staring point of line is the bot's position
                        draw_pos_y,
                        draw_pos_x + cos(getRotationRad()) * 20, // End point of line calculated with polar coordinates
                        draw_pos_y - sin(getRotationRad()) * 20); // make y negative because down is positive on the screen
                        
  Brain.Screen.setFillColor(purple);
  drawPoint(targetX, targetY); // Draw the target point on the screen
  */
}
