
// MATH
double getDistanceTo(double x, double y);
double getClosestAngle(double a);
double getAngleToPos(double x, double y);
void updatePosition();

extern double globalX;
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