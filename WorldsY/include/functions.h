/*
    THIS IS A FILE FOR DECLARING ALL YOUR FUNCTIONS. YOU NEED TO DECLARE THEM HERE SO YOU CAN USE THEM IN YOUR AUTONOMOUS AND OTHER PLACES
    You NEED to have this file that has all the functions you plan on REFERENCING IN OTHER FILES declared here so you can actually refernce them in other files like autonomous.cpp 
      You dont need to do anything more than declare the function signature you'll actually implement these functions in functions.cpp
  NOTE: Essentially you only need to put functions in here that you plan on calling outside the functions.cpp file If you want to create functions that you then use in your 
    main.cpp then you should list it below
  NOTE: These functions are functions you plan on referencing in other files
        Recommended Approach:
          I would recommend that you put all function signatures in here and then in functions.cpp you can actually implement those functions.
*/


// MATH
extern const double toRadians;
extern const double toDegrees;
double keepInRange(double n, double bottom, double top);
double inchesToTicks(double inches);
double ticksToInches(double ticks);

// SENSORS
double getTotalDistance();
void resetTotalDistance();
double getRotationDeg();
double getRotationRad();

// BASIC MOVEMENT
void setLeftBase(double speed);
void setRightBase(double speed);
void setBase(double speed);
void stopBase();
void holdBase();
void fwdConst(double inches, double speed, int timeout=99999);
void revConst(double inches, double speed, int timeout=99999);
void turnL(double inches, double speed);
void turnR(double inches, double speed);
void curveL(double inches, double Lspeed, double Rspeed);
void curveR(double inches, double Lspeed, double Rspeed);

// FAKE PID
void forwardInches(double inches, int maxSpeed); // fake PID. speeds up and slows down
void forwardInchesTimed(double inches, int maxSpeed, int maxTimeMs); // specify time for PID if stuck against wall
void gyroTurn(double targetAngle, int maxSpeed); // fake PID turn
//void gyroTurnSlow(double targetAngle, int maxSpeed); // fake PID turn
/*
void fwdLerp(double targetInches, double startSpeed, double endSpeed);
void revLerp(double inches, double startSpeed, double endSpeed);
void turnLerp(double targetAngleRelative, double startSpeed, double endSpeed);*/

// PID
void forwardPID(double targetInches, double maxSpeed, int timeoutMillis=-1); // Move accorrding to PID. Use for auton
void turnPID(double targetDeg, double maxSpeed, int timeoutMillis=-1); // Move according to PID. Use for auton
void turnPIDSmall(double targetDeg, double maxSpeed, int timeoutMillis=-1); // Move according to PID. Use for auton
void forwardPIDGradual(double targetInches, double maxSpeed, double rate=0.75, int timeoutMillis=-1); // Move accorrding to PID. Use for auton
void curvePID(double targetInches, double maxSpeed, double leftMult, int msTimeout=-1);

void forwardPIDStraight(double targetInches, double maxSpeed, double targetAngle, int msTimeout=-1);