
// MATH
extern double toRadians;
extern double toDegrees;
double keepInRange(double n, double bottom, double top);
double inchesToTicks(double inches);
double ticksToInches(double ticks);

// SENSORS
double getRightReading();
double getLeftReading();
double getTotalDistance();
void resetTotalDistance();
double getRotationDeg();
double getRotationRad();

// BASIC MOVEMENT
void setLeftBase(double speed);
void setRightBase(double speed);
void setBase(double speed);
void stopBase();

// PID
double fwdPIDCycle(double targetDist, double maxSpeed); // Get speed from one cycle of PID
double turnPIDCycle(double targetDegree, double maxSpeed); // Get speed from one cycle of PID
void forwardPID(double targetInches, double maxSpeed, int timeoutMillis = -1); // Move accorrding to PID. Use for auton
void turnPID(double targetDeg, double maxSpeed, int timeoutMillis=-1); // Move according to PID. Use for auton