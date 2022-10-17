/*
  This file is a header for our odometry functions. In addition to functions.h which sets up our normal functions,
  we have an odometry system in odom.h and odom.cpp.
  
  All the odometry code is in a separate file in case someone wants to copy our normal functions without odometry.
  Odometry can be seen as an add-on to the normal functions.

  Odometry is a way to track the x y coordinates of the robot. Instead of making relative commands such as "move forward",
  "turn 90 degrees", you tell the robot to move or turn towards a point.
  
  The robot starts at (0, 0) and facing 0 degrees. We use encoders for tracking distance and the inertial sensor for rotation

*/

// MATH
double getDistanceTo(double x, double y);
double angleWrap(double a);
double getDegToPoint(double x, double y);

void updatePosition(); // call in a separate task from auton to update the position
double getGlobalX();
double getGlobalY();

/* AUTON

  Moving in auton with odometry involves setting the target position and moving towards it with a certain maximum forward speed
    and maximum turning speed.
  If the robot needs to turn a lot to the point, make the turning speed higher. If the robot
    is already close to facing the point, make the turning speed around 5% just so it can make small adjustments.
  If forward speed is too high and turning speed is too low, then the robot can get stuck orbiting the target point,
    so make turning speed a bit higher than you need.

    Here is an example of moving to a point.
        
      setTarget(12, 3); // set target position to (12, 3) on the field
      moveToTarget(25, 30); // move to target position with 25% forward speed and 30% turning speed

*/
void setTarget(double x, double y); // set the target point
void turnToTarget(double maxTurnSpeed); // turn towards the target point
void moveToTarget(double maxFwdSpeed, double maxTurnSpeed);
void moveToTargetRev(double maxFwdSpeed, double maxTurnSpeed);
void passTarget(double maxFwdSpeed, double maxTurnSpeed);
void passTargetRev(double maxFwdSpeed, double maxTurnSpeed);

// DISPLAY
void odomDisplay(); // Visually display position and rotation on the field
void drawPoint(double x, double y); // Draw a point on the odometry dashboard
double getScreenTouchX(); // Get the corresponding coordiantes of where you tap the screen
double getScreenTouchY(); // Robot can move to point that you tap for debugging