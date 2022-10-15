
// MATH
double getDistanceTo(double x, double y);
double getClosestDeg(double a);
double getDegToPoint(double x, double y);
void updatePosition();

extern double globalX; // make these variables accessible outside odom.cpp
extern double globalY;

// AUTON
void setTarget(double x, double y);
void turnToTarget(double maxTurnSpeed);
void moveToTarget(double maxFwdSpeed, double maxTurnSpeed);
void moveToTargetRev(double maxFwdSpeed, double maxTurnSpeed);
void passTarget(double maxFwdSpeed, double maxTurnSpeed);
void passTargetRev(double maxFwdSpeed, double maxTurnSpeed);

// DISPLAY
void odomDisplay(); // Display the odometry dashboard
void drawPoint(double x, double y);