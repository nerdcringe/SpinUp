#include "vex.h"

using namespace vex;


// NEED: fwdRamp, turnRamp, constFwd, pneumatics, rollers/optical sensor


// CONSTANTS //

#define PI 3.14159265
const double toRadians = PI / 180.0; // multiply degrees by this to convert to radians
const double toDegrees = 180.0 / PI; // multiply radians by this to convert to degrees

//const double WHEEL_CIRCUMFERENCE = 3.25 * PI; // Circumference of powered wheels (diameter * PI)
const double TRACKING_CIRCUMFERENCE = 2.75 * PI; // Circumference of tracking wheels (diameter * PI)


//bool exitLoop = false; // Exit a movement loop if robot is stuck and encoders don't detect movement


// MATH FUNCTIONS /////////////////////////////////////////////

// Return the number clamped between two numbers
double keepInRange(double n, double bottom, double top) {
  if (n < bottom)
    n = bottom;
  if (n > top)
    n = top;
  return n;
}


// Convert between inches and ticks of tracking wheels using dimensional analysis
double inchesToTicks(double inches)
{
  return inches * (360 / TRACKING_CIRCUMFERENCE);
}

double ticksToInches(double ticks)
{
  return ticks * (TRACKING_CIRCUMFERENCE / 360);
}

// SENSOR FUNCTIONS ///////////////////////////////////////////////////////

// Old motor encoder distance tracking
/*
double getTotalDistance() // return average motor encoder value
{
  double sum = L1BASE.rotation(deg)
             + L2BASE.rotation(deg)
             + L3BASE.rotation(deg)
             + L4BASE.rotation(deg)
             + R1BASE.rotation(deg)
             + R2BASE.rotation(deg)
             + R3BASE.rotation(deg)
             + R4BASE.rotation(deg);
  return ticksToInches(sum) / 8;
}

// reset motor encoders
void resetTotalDistance()
{
  
  R1BASE.resetRotation();
  R2BASE.resetRotation();
  R3BASE.resetRotation();
  R4BASE.resetRotation();

  L1BASE.resetRotation();
  L2BASE.resetRotation();
  L3BASE.resetRotation();
  L4BASE.resetRotation();
}*/

// Obtain the current values of the tracking wheel encoders
double getRightReading() {
  return -ticksToInches(encoderR.rotation(deg)); // negative because right encoder is backwards
}

double getLeftReading() {
  return -ticksToInches(encoderL.rotation(deg));
}


/*
// Check if position is not changing so robot doesn't get stuck trying to drive past a wall (todo)
bool isStopped()
{
  return fabs(deltaRight) < 0.001 && fabs(deltaLeft) < 0.001;
}*/


// To track distance (even outside of odometry) we're using the tracking wheels instead of motor encoders
// Since we shouldn't reset the tracking wheels entirely for odometry to keep working
// It just subtracts the encoderResetValue to zero it out when you call resetTotalDistance()

double encoderResetValue = 0; // Used to zero out motor encoders

// Return the average tracking wheel distance
double getTotalDistance()
{
  return (getLeftReading() + getRightReading())/2 - encoderResetValue;
}

// Clear the total distance accumulated in the beginning of each movement
void resetTotalDistance()
{
  encoderResetValue = (getLeftReading() + getRightReading())/2;
}


// Obtain the current inertial sensor rotation relative to the starting rotation
double getRotationDeg()
{
  return -INERTIAL.rotation(deg); // Important that this is negative
  // This makes positive angles CCW (left) and negative angles CW (right)
  // 0 degrees is on the positive x-axis
}

// Obtain the current inertial sensor rotation in radians
// A full rotation is 2 PI instead of 360
// Trig functions natively use radians
double getRotationRad()
{
  return getRotationDeg() * toRadians;
}



// BASIC MOVEMENT ////////////////////////////

// Set the speeds of different sides of the base
void setLeftBase(double speed)
{
  L1BASE.spin(fwd, speed, pct);
  L2BASE.spin(fwd, speed, pct);
  L3BASE.spin(fwd, speed, pct);
  L4BASE.spin(fwd, speed, pct);
}

void setRightBase(double speed)
{
  R1BASE.spin(fwd, speed, pct);
  R2BASE.spin(fwd, speed, pct);
  R3BASE.spin(fwd, speed, pct);
  R4BASE.spin(fwd, speed, pct);
}

void setBase(double speed)
{
  setLeftBase(speed);
  setRightBase(speed);
}

void stopBase()
{
  L1BASE.stop(coast);
  L2BASE.stop(coast);
  L3BASE.stop(coast);
  L4BASE.stop(coast);

  R1BASE.stop(coast);
  R2BASE.stop(coast);
  R3BASE.stop(coast);
  R4BASE.stop(coast);
}




// PID /////////////////////////////////////////////////////////////////////

/*
  PID is a way to move some distance or turn to some angle.
  The distance or angle from a target is called error.

  To reduce error precisely, PID uses three variables to calculate speed.
  It adds up the three variables and uses the sum as the speed output.

    P: Proportional
      - If you’re not where you want to be, get there. Go faster if you're far and slower if you're close.
      - Where most of the speed comes from.
    
    I: Integral
      - If you haven’t been where you want to be for a long time, get there faster.
      - Makes sure speed is fast enough to overcome friction & weight of robot (steady-state error).

    D: Derivative
      - If you’re getting close to where you want to be, slow down.
      - Smooths out any unwanted oscillations and overshoot.

  PID is tuned by changing the values Kp, Ki, and Kd (found in the PID cycle functions).
  Each tuning constant controls how much its corresponding variable affects the speed output.
*/

double fwd_error = 0; // Distance from target forward distance
double fwd_lastError = 0; // Keep track of last error for the derivative (rate of change)
double fwd_integral = 0; // Integral accumulates the error, speeding up if target is not reached fast enough
double fwd_derivative = 0; // Derivative smooths out oscillations and counters sudden changes

double turn_error = 0; // Relative angle from the target rotation (degrees)
double turn_lastError = 0;
double turn_integral = 0;
double turn_derivative = 0;




// Get speed by doing one cycle of PID
double fwdPIDCycle(double targetDist, double maxSpeed)
{ 
  double Kp = 5; // Proportional makes the speed proportional to the error

  double Ki = 0.01; // Integral accumulates error to the speed over time
  // Integral is often used to to overcome friction at the end due to derivative

  double Kd = 1;//45.0; // Derivative slows down the speed if it is too fast

  // Don't let the integral term have too much control
  double integralPowerLimit = 40 / Ki; // 40 is the maximum power that integral can contribute
  double integralActiveZone = 10; // Only start accumulating integral
  double errorThreshold = 0.25; // Exit loop when error is less than this

  double speed = 0;

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

    // make sure speed is always enough to overcome steady-state error
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
double turnPIDCycle(double targetDegree, double maxSpeed)
{
  double Kp = 0.56;
  double Ki = 0.00075;//0.01;
  double Kd = 0.5;//1;
  double integralPowerLimit =
      40 / Ki;                   // little less than half power
  double integralActiveZone = 15; // degrees to start accumulating to integral
  double errorThreshold = 0.5; // Exit loop when error is less than this

  double speed = 0;

  turn_error = targetDegree - getRotationDeg(); // The error is the relative angle to the target angle

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


// Run a closed PID loop for linear distance in inches
void forwardPID(double targetInches, double maxSpeed, int timeoutMillis = -1) // default timeout is -1, meaning unlimited
{
  double speed = 1;

  timer Timer;
  Timer.reset(); // start timer for timeout

  resetTotalDistance();
  // While time isn't up or time isn't set (default value of -1)
  while (speed != 0 && (Timer.time() < timeoutMillis || timeoutMillis == -1))
  {
    double currentDist = targetInches - getTotalDistance(); // calculate the distance from target

    speed = fwdPIDCycle(currentDist, maxSpeed); // plug in distance and speed into PID
    setLeftBase(speed);
    setRightBase(speed);

    task::sleep(10);
  }
  stopBase();
}


// Run a closed PID loop for turning to a specific absolute degree
// Setting targetDeg to 0 degrees always returns to the starting angle
void turnPID(double targetDeg, double maxSpeed, int timeoutMillis=-1) // default timeout is -1, meaning unlimited
{
  double speed = 1;

  timer Timer;
  Timer.reset(); // start timer for timeout

  resetTotalDistance();
  // While time isn't up or time isn't set (default value of -1)
  while (speed != 0 && (Timer.time() < timeoutMillis || timeoutMillis == -1))
  {
    speed = turnPIDCycle(targetDeg, maxSpeed);
    setLeftBase(-speed);
    setRightBase(speed);

    task::sleep(10);
  }
  stopBase();
}