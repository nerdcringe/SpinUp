/*
    THIS IS A FILE FOR DECLARING ALL YOUR FUNCTIONS. YOU NEED TO DECLARE THEM HERE SO YOU CAN USE THEM IN YOUR AUTONOMOUS AND OTHER PLACES
    You NEED to have this file that has all the functions you plan on REFERENCING IN OTHER FILES declared here so you can actually refernce them in other files like autonomous.cpp 
      You dont need to do anything more than declare the function signature you'll actually implement these functions in functions.cpp
  NOTE: Essentially you only need to put functions in here that you plan on calling outside the functions.cpp file If you want to create functions that you then use in your 
    main.cpp then you should list it below
  NOTE: These functions are functions you plan on referencing in other files
        Recommended Approach:
          I would recommend that you put all function signatures in here and then in functions.cpp you can actually implement those functions.
  The general way to declare a function is as follows:
      - void kachiga(int power);
  Breaking down each word:
      - "void"      <-- is the return type. We have many of these (ex: int, double, float, bool, void)
      - "kachiga"   <-- is the name of the function that we are declaring this will be the name you call when referncing the function for the rest of the code
      - "int power" <-- is a parameter, your functions can have as many of those as it needs as long as you give it a type and name (ex: float distance, bool reversed, int speed)
    BELOW ARE A FEW EXAMPLES OF Function DECLARATIONS
*/


// MATH
extern const double toRadians;
extern const double toDegrees;
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
void forwardPID(double targetInches, double maxSpeed, int timeoutMillis); // Move accorrding to PID. Use for auton
void turnPID(double targetDeg, double maxSpeed, int timeoutMillis); // Move according to PID. Use for auton


// CATAPULT



// PNEUMATICS


