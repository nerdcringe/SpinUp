#include "vex.h"

using namespace vex;


// NEED: fwdRamp, turnRamp, constFwd, pneumatics, rollers/optical sensor


// CONSTANTS //

const bool pto_6m_val = 0;
const bool pto_8m_val = 1;


#define PI 3.14159265
const double toRadians = PI / 180.0; // multiply degrees by this to convert to radians
const double toDegrees = 180.0 / PI; // multiply radians by this to convert to degrees

//const double WHEEL_CIRCUMFERENCE = 3.25 * PI; // Circumference of powered wheels (diameter * PI)
const double TRACKING_CIRCUMFERENCE = 2.75 * PI; // Circumference of tracking wheels (diameter * PI)

const double cataResetAngle = 45;



double degree = 0.0;
double KPC = 50.0;
double KDC = 0.12;
double correctionL = 0;
double correctionR = 0;


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
double inchesToTicks(double inches) {
  return inches * (360 / TRACKING_CIRCUMFERENCE) * change;
}

double ticksToInches(double ticks) {
  return ticks * (TRACKING_CIRCUMFERENCE / 360) * change;
}

// SENSOR FUNCTIONS ///////////////////////////////////////////////////////


double getTotalDistance() // return average motor encoder value
{
  // skip L2 cuz thats the roller/intake and R2 cuz thats the cata

  return ticksToInches(
    L1BASE.rotation(deg) +
    L2BASE.rotation(deg) +
    L3BASE.rotation(deg) +
    R1BASE.rotation(deg) + 
    R2BASE.rotation(deg) + 
    R3BASE.rotation(deg)) / 6;
}
// - encoderResetValue;

// reset motor encoders
void resetTotalDistance()
{
  R1BASE.resetRotation();
  R2BASE.resetRotation();
  R3BASE.resetRotation();

  L1BASE.resetRotation();
  L2BASE.resetRotation();
  L3BASE.resetRotation();
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
}

void setRightBase(double speed)
{
  R1BASE.spin(fwd, speed, pct);
  R2BASE.spin(fwd, speed, pct);
  R3BASE.spin(fwd, speed, pct);
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

  R1BASE.stop(coast);
  R2BASE.stop(coast);
  R3BASE.stop(coast);
}




void holdBase()
{
  L1BASE.stop(hold);
  L2BASE.stop(hold);
  L3BASE.stop(hold);

  R1BASE.stop(hold);
  R2BASE.stop(hold);
  R3BASE.stop(hold);
}


void forwardConst(double inches, double speed)
{
    resetTotalDistance();
    task::sleep(15);
    if (inches < 0) { speed *=-1; }
    setBase(speed);

    while(fabs(getTotalDistance()) < inches)
    {
      task::sleep(10);
    //controllerPrim.Screen.setCursor(1,1);
    //controllerPrim.Screen.clearLine();
      //controllerPrim.Screen.print("%f", encoderInches);
    }

    stopBase();
    /*controllerPrim.Screen.setCursor(1,1);
    controllerPrim.Screen.clearLine();
    controllerPrim.Screen.print("Done");*/
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
    task::sleep(10);
  }

  stopBase();
  task::sleep(15);
}




void forwardInchesTimed(double inches, int maxSpeed, int maxTimeMs)
{
  resetTotalDistance();
  //maxSpeed = keepInRange(maxSpeed, 0, 100);

  //MinSpeed :  Ensures the movement overcomes friction.
  const double minSpeed = 3;      // Lowest speed the motors will go; Turning is more precise but jankier when lower.

  // Acceleration rates: changes rate of speed up/slow down
  // Gradually speeds up for the first half, slows down for second half
  const double accelRate = 100;    // Speed multiplier while speeding up (slope at start).
  const double deaccelRate = 4;  // Speed multiplier while slowing down (slope at end). Starts slowing down later when higher

  double targetDistance = inches;  // How far the robot should travel
  double targetRange = 0.25;                         // Distance from the desired distance the robot has to be to stop.
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
    task::sleep(10);
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

  double targetRange = 1.0; // Distance from the desired angle that is allowed
  double error = relativeAngle; // Distance from the desired range
  double progress;              // Degrees the robot has turned already
  double minSpeed = 2; // Lowest speed the motors will go; Turning is generally more precise when lower, but slower.
  double deaccelRate = 1.0;//3; // 2.4

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
    if (error < 0) {
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
     
    task::sleep(10);
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

/// New PID Stuff///////////////

void forwardPID(float targetInches, float maxPower, int msTimeout) {
  float change = 1.34;
  float Kp = 0.3031; // you need to tune these value manually. 0.305
  float Ki = 0.008;// //.03; // they are just arbitrary constants so you need to
                     //test     0.0145
  float Kd = 0.53231; //.18922175;    //until they work. 0.529

  float error =
      inchesToTicks(targetInches * change) - getTotalDistance(); // desired - actual
  float lastError;
  float integral;
  float derivative;

  float integralPowerLimit =
      40 / Ki; // little less than half power in pct (percent)
  float integralActiveZone = 15;
  // explaining integral active zone
  // area where proportion is effective is represented with >>/<<
  // area where proportion is not strong enough to move base rep with ----
  // 0 is the target where error is equal to 0
  // only want integral during ineffective zone to prevent windup of power
  //>>>>>>>>>>>>-----------0---------------<<<<<<<<<<<<<<<<

  float exitThreshold = 0.5; // Exit loop when error is less than this
  float finalPower;

  resetTotalDistance();
  Brain.resetTimer();

  while (fabs(error) > exitThreshold &&
         Brain.timer(vex::timeUnits::msec) < msTimeout) {
    // Brain.Screen.printAt(140, 85,"ROTATION: %.3f deg", getRotation());
    error = inchesToTicks(targetInches * change) - getTotalDistance();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;
    /*
     if (error == 0)
     {
       derivative = 0;
     }*/

    finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    finalPower = keepInRange(finalPower, -maxPower, maxPower);

    setLeftBase(finalPower);
    setRightBase(finalPower);
    Brain.Screen.printAt(140, 25, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
                         (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(
        140, 50, "Dist: %.2f, Error: %.2f", ticksToInches(getTotalDistance()),
        error - getTotalDistance());
    vex::task::sleep(40);
  }
  stopBase();
}


void turnPID(float target, float maxPower, int msTimeout) {
  float change = 1.0;
  /*
    float Kp = 0.5105;   //getting to target
    float Ki = 0.00825; // increases speed (builds up over time) before: 0.008
    float Kd = 0.0111;*/    //slow down

  /*float Kp = 0.6;    // 0.508497;
  float Ki = 0.0001; // 0.007;
  float Kd = 0.0504; // 051;//.09;*/

  // new constants for Kalahari
  float Kp = 0.48;       // 0.508497;
  float Ki = 0.005;//75;//19;      // 11; //0.007;
  float Kd = 0.499;//3; // 0.0504;//051;//.09;

  float error = (target * change) - getRotationDeg();
  float lastError;
  float integral;
  float derivative;

  float integralPowerLimit =
      40 / Ki;                   // little less than half power in pct (percent)
  float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks
  // explaining integral active zone
  // area where proportion is effective is represented with >>/<<
  // area where proportion is not strong enough to move base rep with ----
  // 0 is the target where error is equal to 0
  // only want integral during ineffective zone to prevent windup of power
  //>>>>>>>>>>>>-----------0---------------<<<<<<<<<<<<<<<<

  float exitThreshold = 0.75; // Exit loop when error is less than this
  float finalPower;

  resetTotalDistance();
  Brain.resetTimer();

  while (fabs(error) > exitThreshold &&
         Brain.timer(vex::timeUnits::msec) < msTimeout) {
    Brain.Screen.printAt(140, 95, "ROTATION: %.3f deg", getRotationDeg());
    error = (target * change) - getRotationDeg();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;
    /*
     if (error == 0)
     {
       derivative = 0;
     }*/

    finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    finalPower = keepInRange(finalPower, -maxPower, maxPower);

    setLeftBase(-finalPower);
    setRightBase(finalPower);

    // Brain.Screen.printAt(140, 25,"P: %.2f, I: %.2f, D: %.2f", (Kp * error),
    // (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(140, 65, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
                         (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(140, 45, "INERTIAL: %.2f", INERTIAL.value());
    Brain.Screen.printAt(140, 25, "INERTIAL2: %.2f", getRotationDeg());

    vex::task::sleep(40);
  }
  stopBase();
}



/*double fwd_error = 0; // Distance from target forward distance
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
  double Kp = 45;//74; //7; // Proportional makes the speed proportional to the error

  double Ki = 0; //0.02; // Integral accumulates error to the speed over time
  // Integral is often used to to overcome friction at the end due to derivative

  double Kd = 150; //1.45;//45.0; // Derivative slows down the speed if it is too fast

  // Don't let the integral term have too much control
  double integralPowerLimit = 40 / Ki; // 40 is the maximum power that integral can contribute
  double integralActiveZone = 10; // Only start accumulating integral
  double errorThreshold = 0.1; // Exit loop when error is less than this

  double speed = 0;

  fwd_error = targetDist; // Error is the distance from the target

  if (fabs(fwd_error) > errorThreshold || fabs(turn_derivative * Kd) > 0.0001) // Check if error is over the acceptable threshold
  { // don't exit if derivative is too big, cause it should be slow enough to stop precisely
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

    Brain.Screen.printAt(210, 140, "P: %.1f    ", (Kp * fwd_error));
    Brain.Screen.printAt(210,160,"I: %.1f     ", (Ki * fwd_integral));
    Brain.Screen.printAt(210,180,"D: %.1f     ", (Kd * fwd_derivative));
    Brain.Screen.printAt(210, 200, "%f  ", fwd_error);
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
  double Kp = 1.8;//0.4;
  double Ki = 0;//0.0001;//0.005;//4;//0.01;
  double Kd = 2;//0.31;//33;//43;//1;
  double integralPowerLimit =
      5 / Ki;                   // little less than half power
  double integralActiveZone = 5; // degrees to start accumulating to integral
  double errorThreshold = 0.3; // Exit loop when error is less than this

  double speed = 0;

  turn_error = targetDegree - getRotationDeg(); // The error is the relative angle to the target angle

  if (fabs(turn_error) > errorThreshold || fabs(turn_derivative) > 0.1) // only exit if derivative isnt low, meaning speed not changing much
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
    
    if (speed > 0 && speed < 4) {
      speed = 2;
    } if (speed < 0 && speed > -2) {
      speed = -2;
    }
    Brain.Screen.printAt(210, 160, "P: %.1f, I: %.1f, D: %.1f    ", (Kp * turn_error), (Ki * turn_integral), (Kd * turn_derivative));
    controllerPrim.Screen.setCursor(1, 1);
    //controllerPrim.Screen.clearScreen();
    controllerPrim.Screen.print("%.1f %.1f %.1f", (Kp * turn_error), (Ki * turn_integral), (Kd * turn_derivative));
  
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

// Run a closed PID loop for linear distance in inches
void forwardPIDIncr(double targetInches, double maxSpeed, int timeoutMillis) // default timeout is -1, meaning unlimited
{
  double speed = 1;
  timer Timer;
  Timer.reset(); // start timer for timeout
  float currentMax = 6; // start slow ramp up max speed eventually

  resetTotalDistance();
  // While time isn't up or time isn't set (default value of -1)
  while (speed != 0 && (Timer.time() < timeoutMillis || timeoutMillis == -1))
  {
    if (currentMax < maxSpeed)
    {
      currentMax += 1.3; // speed up
    }
    if (currentMax > maxSpeed)
    {
      currentMax = maxSpeed;
    }
    double error = targetInches - getTotalDistance(); // calculate the distance from target

    speed = fwdPIDCycle(error, currentMax); // plug in error and speed into PID
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
*/

// catapult //
 //untested
bool doCataPID = false;

/*int catapultPID()
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
}*/

/*
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
*/

bool resettingCata = false;
int catapultReset()
{
  /*timer exitTimer; ! && ||
  exitTimer.clear();*/

  if (!resettingCata) // only click when not resetting cata yet
  {
    resettingCata = true;
    R2BASE.spin(reverse, 100, pct); // start resetting cata
    
    while (!CATALIMIT.pressing() && resettingCata){// && PTO.value() == pto_6m_val){ 
      // reload to limit switchs
      wait(10, msec);
      /*if (exitTimer.time(msec) > 5000){
        break; // exit after 5 seconds by default
      }*/
    }
    resettingCata = false;
    R2BASE.stop();
  }
  return 0;
}

void catapultResetDriver(){
  catapultReset();
}





void toggleLock() {
  LOCK.set(!LOCK.value()); // invert the value
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


void roller(double distance, double speed)
{
  L2BASE.rotateFor(reverse, distance, deg,speed,velocityUnits::pct);
}

void setIntake(double speed){
  L2BASE.spin(fwd,speed,velocityUnits::pct);
}


// deploy endgame mech
void manspread()
{
  ENDGAME.set(!ENDGAME.value());
}