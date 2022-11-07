#include "vex.h"

using namespace vex;


// NEED: fwdRamp, turnRamp, constFwd, pneumatics, rollers/optical sensor


// CONSTANTS //

#define pto_6m_val 1
#define pto_8m_val 0


#define PI 3.14159265
const double toRadians = PI / 180.0; // multiply degrees by this to convert to radians
const double toDegrees = 180.0 / PI; // multiply radians by this to convert to degrees

//const double WHEEL_CIRCUMFERENCE = 3.25 * PI; // Circumference of powered wheels (diameter * PI)
const double TRACKING_CIRCUMFERENCE = 2.75 * PI; // Circumference of tracking wheels (diameter * PI)

const double cataResetAngle = 45;


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


double change = 1.0;// change distance cuz gear ratio

// Convert between inches and ticks of tracking wheels using dimensional analysis
double inchesToTicks(double inches)
{
  return inches * (360 / TRACKING_CIRCUMFERENCE) * change;
}

double ticksToInches(double ticks)
{
  return ticks * (TRACKING_CIRCUMFERENCE / 360) * change;
}

// SENSOR FUNCTIONS ///////////////////////////////////////////////////////



// Obtain the current values of the tracking wheel encoders
double getRightReading() {
  return ticksToInches(R3BASE.rotation(deg));
  //return -ticksToInches( (R3BASE.rotation(deg) + R4BASE.rotation(deg) ) / 2);
  //return -ticksToInches(encoderR.rotation(deg));
}

double getLeftReading() {
  return ticksToInches(L3BASE.rotation(deg));
  //return -ticksToInches( (L3BASE.rotation(deg) + L4BASE.rotation(deg) ) / 2);
  //return -ticksToInches(encoderL.rotation(deg));
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

// /double encoderResetValue = 0; // Used to zero out motor encoders

// Old motor encoder distance tracking

double getTotalDistance() // return average motor encoder value
{
  // skip L2 cuz thats the roller/intake and R2 cuz thats the cata

  return ticksToInches(
    L1BASE.rotation(deg) +
    R1BASE.rotation(deg)) / 2;
}
// - encoderResetValue;

// reset motor encoders
void resetTotalDistance()
{
  R1BASE.resetRotation();
  //R3BASE.resetRotation();
  //R4BASE.resetRotation();

  L1BASE.resetRotation();
  //L3BASE.resetRotation();
  //L4BASE.resetRotation();
}
  /*float sum = L1BASE.rotation(deg)
             + L3BASE.rotation(deg)
             + L4BASE.rotation(deg)
             + R1BASE.rotation(deg)
             + R3BASE.rotation(deg)
             + R4BASE.rotation(deg);
  encoderResetValue = (ticksToInches(sum) / 6);
  //encoderResetValue = getTotalDistance();
}*/


/*
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
*/




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
  if (PTO.value() == 0) { // If pto in - 8 motor
    L2BASE.spin(fwd, speed, pct);
  }
  L3BASE.spin(fwd, speed, pct);
  L4BASE.spin(fwd, speed, pct);
}

void setRightBase(double speed)
{
  R1BASE.spin(fwd, speed, pct);
  if (PTO.value() == 0) { // If pto in - 8 motor
    R2BASE.spin(fwd, speed, pct);
  }
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



// Speed up and slow down gradually
// Use negative distance to go backwards
// More info: https://www.vexforum.com/t/advanced-pid-and-motion-profile-control/28400/3
void forwardInches(double inches, int maxSpeed)
{
  resetTotalDistance();
  //maxSpeed = keepInRange(maxSpeed, 0, 100);

  //MinSpeed : Ensures the movement overcomes friction.
  const double minSpeed = 3;      // Lowest speed the motors will go; Turning is more precise but jankier when lower.

  // Acceleration rates: changes rate of speed up/slow down
  // Gradually speeds up for the first half, slows down for second half
  const double accelRate = 100;    // Speed multiplier while speeding up (slope at start).
  const double deaccelRate = 4;  // Speed multiplier while slowing down (slope at end). Starts slowing down later when higher

  double targetDistance = inches;/*inchesToTicks(inches)*/;  // How far the robot should travel
  double targetRange = .25;                         // Distance from the desired distance the robot has to be to stop.
  double error = targetDistance;                  // Distance from the desired distance
  double speed;                                   // Actual speed value of the motors

  while (fabs(error) > targetRange) {

    error = targetDistance - getTotalDistance(); // Error = desired - actual

    // First half: accelerate to max. Second half, deccelerate to min
      // Speed is proportional to the error/target distance.
      // Since error is always decreasing, the speed proportion is inverted (subtracted from 1) for the first half of distance
      // travelled so speed increases proportionally instead of decreasing. 
      // fabs() (absolute value) is used because the direction is determined by the sign of the error, not of the speed.
    if (fabs(error) > fabs(targetDistance/2))
    {
      speed = (1 - (fabs(error) / fabs(targetDistance))) * accelRate; // Speeds up as error decreases
    }
    else
    {
      speed = (fabs(error) / fabs(targetDistance)) * deaccelRate;  // Slows down as error increases
    }
    speed = fabs(speed);
    speed *= maxSpeed;
    speed = keepInRange(speed, minSpeed, maxSpeed);

    // Go forward if positive error and backwards if negative error
    if (error < 0)
    {
      speed *= -1;
    }
    setLeftBase(speed);
    setRightBase(speed);

    Brain.Screen.printAt(1, 60, "Target Dist: %.2f  ", (targetDistance));
    Brain.Screen.printAt(1, 80, "Progress:    %.2f  ", (getTotalDistance()));
    Brain.Screen.printAt(1, 120,"Error:       %.2f      ", error);
    Brain.Screen.printAt(1, 100,"Speed:       %.2f  ", speed);
  }

  stopBase();
  task::sleep(15);
}




void forwardInchesTimed(double inches, int maxSpeed, int maxTimeMs)
{
  resetTotalDistance();
  //maxSpeed = keepInRange(maxSpeed, 0, 100);

  //MinSpeed :  Ensures the movement overcomes friction.
  const double minSpeed = 2;      // Lowest speed the motors will go; Turning is more precise but jankier when lower.

  // Acceleration rates: changes rate of speed up/slow down
  // Gradually speeds up for the first half, slows down for second half
  const double accelRate = 30;    // Speed multiplier while speeding up (slope at start).
  const double deaccelRate = 8;  // Speed multiplier while slowing down (slope at end). Starts slowing down later when higher

  double targetDistance = inches;  // How far the robot should travel
  double targetRange = 3;                         // Distance from the desired distance the robot has to be to stop.
  double error = targetDistance;                  // Distance from the desired distance
  double speed;                                   // Actual speed value of the motors

  timer Timer;
  Timer.clear();

  while (fabs(error) > targetRange && Timer.time(msec) <= maxTimeMs) {

    error = targetDistance - getTotalDistance(); // Error = desired - actual

    // First half: accelerate to max. Second half, deccelerate to min
      // Speed is proportional to the error/target distance.
      // Since error is always decreasing, the speed proportion is inverted (subtracted from 1) for the first half of distance
      // travelled so speed increases proportionally instead of decreasing. 
      // fabs() (absolute value) is used because the direction is determined by the sign of the error, not of the speed.
    if (fabs(error) > fabs(targetDistance/2))
    {
      speed = (1 - (fabs(error) / fabs(targetDistance))) * accelRate; // Speeds up as error decreases
    }
    else
    {
      speed = (fabs(error) / fabs(targetDistance)) * deaccelRate;  // Slows down as error increases
    }
    speed = fabs(speed);
    speed *= maxSpeed;
    speed = keepInRange(speed, minSpeed, maxSpeed);

    // Go forward if positive error and backwards if negative error
    if (error < 0)
    {
      speed *= -1;
    }
    setLeftBase(speed);
    setRightBase(speed);

    /*Brain.Screen.printAt(1, 60, "Target Dist: %.2f", ticksToInches(targetDistance));
    Brain.Screen.printAt(1, 80, "Progress:    %.2f", ticksToInches(encoderAverage()));
    Brain.Screen.printAt(1, 120,"Error:       %.2f ticks, %.2f \"inches\"", error, ticksToInches(error));
    Brain.Screen.printAt(1, 100,"Speed:       %.2f", speed);*/
    Brain.Screen.printAt(1, 60, "Target Dist: %.2f  ", (targetDistance));
    Brain.Screen.printAt(1, 80, "Progress:    %.2f  ", (getTotalDistance()));
    Brain.Screen.printAt(1, 120,"Error:       %.2f      ", error);
    Brain.Screen.printAt(1, 100,"Speed:       %.2f  ", speed);
  }

  stopBase();
  task::sleep(15);
}


// Absolute turn in degrees
void gyroTurn(double targetAngle, int maxSpeed)
{
  double initialAngle = getRotationDeg();
  //          <---------------------------|------------------->
  // Negative Degrees (Counterclockwise),    Positive Degrees (Clockwise)
  double relativeAngle = targetAngle - getRotationDeg();

  double targetRange = 2; // Distance from the desired angle that is allowed
  double error = relativeAngle; // Distance from the desired range
  double progress;              // Degrees the robot has turned already
  double minSpeed = 5; // Lowest speed the motors will go; Turning is generally more precise when lower, but slower.
  double deaccelRate = 2.5;//3; // 2.4

  double speed; // Actual speed value of the motors

  maxSpeed = keepInRange(maxSpeed, 0, 100);

  while (fabs(error) > targetRange) {
    progress = getRotationDeg() - initialAngle;
    error = progress - relativeAngle;

    // Speed starts at maximum and approaches minimum as the gyro value
    // approaches the desired angle. It deccelerates for precision.
    speed = fabs(error / relativeAngle); // Speed is absolute valued so that it can be kept in range properly
    speed *= maxSpeed * deaccelRate;
    speed = keepInRange(speed, minSpeed, maxSpeed);
    
    // Keep speed either above maxSpeed or below negative maxSpeed so it's fast enough to overcome friction
    if (error > 0) {
      setLeftBase(-speed);
      setRightBase(speed);
    } else {
      setLeftBase(speed);
      setRightBase(-speed);
    }

    /*controllerPrim.Screen.clearScreen();
    task::sleep(10);
    controllerPrim.Screen.setCursor(1, 1);
    task::sleep(10);
    controllerPrim.Screen.print("%f", getRotation());*/

     Brain.Screen.printAt(1, 120, "Desired angle: %.2f, Relative angle: %.2f",
     degrees, relativeAngle);
     Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);
     Brain.Screen.printAt(1, 180, "Gyro: %.2f", getRotationDeg());
  }

  stopBase();
  task::sleep(15);
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
  double Kp = 8; // Proportional makes the speed proportional to the error

  double Ki = 0.015; // Integral accumulates error to the speed over time
  // Integral is often used to to overcome friction at the end due to derivative

  double Kd = 1.4;//45.0; // Derivative slows down the speed if it is too fast

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
  double Ki = 0.004;//0.01;
  double Kd = 0.43;//1;
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
void forwardPID(double targetInches, double maxSpeed, int timeoutMillis) // default timeout is -1, meaning unlimited
{
  double speed = 1;
  timer Timer;
  Timer.reset(); // start timer for timeout

  resetTotalDistance();
  // While time isn't up or time isn't set (default value of -1)
  while (speed != 0 && (Timer.time() < timeoutMillis || timeoutMillis == -1))
  {
    double error = targetInches - getTotalDistance(); // calculate the distance from target

    speed = fwdPIDCycle(error, maxSpeed); // plug in error and speed into PID
    setLeftBase(speed);
    setRightBase(speed);

    task::sleep(10);
  }
  stopBase();
}


// Run a closed PID loop for turning to a specific absolute degree
// Setting targetDeg to 0 degrees always returns to the starting angle
void turnPID(double targetDeg, double maxSpeed, int timeoutMillis) // default timeout is -1, meaning unlimited
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


// catapult //
 //untested
bool doCataPID = false;

int catapultPID()
{
  double Kp = 0.5; // proportion between error and speed
  double Kd = 0.0; // proportion between error and speed
  int minSpeed = 3; // motor only moves when going at least 2%ish
  int maxSpeed = 20;
  
  double error = 0;
  double cata_lastError = error;
  double derivative = 0;

  while (true)
  {
  Brain.Screen.printAt(210, 220, "%d", doCataPID);
    while (doCataPID && PTO.value() == 1) // only repeat if not reset and if pto is using cata
    {
      error = cataResetAngle - CATAPOT.angle();
      derivative = error - cata_lastError;

      // sum up the tuned PID variables to obtain a speed value
      double speed = (Kp * error) + (Kd * derivative); // speed is proportional to error (P) and smooths out oscillations (D)

      // make sure speed doesn't exceed maxSpeed
      if (speed > maxSpeed) {
        speed = maxSpeed;
      } else if (speed < -maxSpeed) {
        speed = -maxSpeed;
      }

      if (speed > 0 && speed < minSpeed) {
        // if going forward, make speed at least minSpeed in positive direction
        speed = minSpeed;
      } else if (speed < 0 && speed > -minSpeed) {
        // if going reverse, make speed at least minSpeed in negative direction
        speed = -minSpeed;
      }

      R2BASE.spin(fwd, speed, pct);
      task::sleep(15);

      if (fabs(error) < 1) // when within this distance of resetAngle, exit the loop
      {
        L1BASE.stop();
        doCataPID = false;
        Brain.Screen.printAt(210, 200, "Done PID");
      }

    } // while (doCataPID)

    task::sleep(25); // wait to give cpu a rest
  } // while (true)
  return 0;
}


// untested
int catapultFire()
{
  Brain.Screen.printAt(210, 130, "start fire");
  doCataPID = false;
  wait(100, msec);
  while (CATAPOT.angle() > 20 && PTO.value() == 1) // while cata rotation is still high (before it launches)
  {
        Brain.Screen.printAt(210, 150, "waiting fire");
    R2BASE.spin(fwd, 15, pct); // bring down cata to fire
  }
  R2BASE.stop();
  wait(500, msec);
        Brain.Screen.printAt(210, 170, "done fire");
  doCataPID = true;

  return 0;
}

void catapultFireAsync()
{
  task f(catapultFire); // fire in a separate task
}
\

bool resettingCata = false;
void catapultReset()
{
  if (!resettingCata) // only click when not resetting cata yet
  {
    resettingCata = true;
    R2BASE.spin(reverse, 100, pct); // start esetting cata

    while (!CATALIMIT.pressing() && resettingCata){// && PTO.value() == pto_6m_val){ 
      // reload to limit switchs
      wait(10, msec);
    }
    resettingCata = false;
    R2BASE.stop();
  }
}



// pneumatics //

void pto6(){
  PTO.set(pto_6m_val);
}

void pto8(){
  PTO.set(pto_8m_val);
}

void togglePto()
{  
  //controllerPrim.Screen.print("%d", PWT.value());
  if (PTO.value() == pto_6m_val) // if out - 6 motor
  {
    pto8(); // set to in - 8 motor
  }
  else // else it's in - 8 motor
  {
    pto6(); // set to out - 6 motor
  }
}
