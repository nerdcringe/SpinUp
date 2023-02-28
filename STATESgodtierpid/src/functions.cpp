#include "vex.h"
using namespace vex;

// CONSTANTS //
#define PI 3.14159265
const double toRadians = PI / 180.0; // multiply degrees by this to convert to radians
const double toDegrees = 180.0 / PI; // multiply radians by this to convert to degrees

//const double WHEEL_CIRCUMFERENCE = 3.25 * PI; // Circumference of powered wheels (diameter * PI)
const double TRACKING_CIRCUMFERENCE = 2.75 * PI; // Circumference of tracking wheels (diameter * PI)

/*
double degree = 0.0;
double KPC = 50.0;
double KDC = 0.12;
double correctionL = 0;
double correctionR = 0;*/

// MATH FUNCTIONS /////////////////////////////////////////////

  // Return the number clamped between two numbers
  double keepInRange(double n, double bottom, double top) {
    if (n < bottom)
      n = bottom;
    if (n > top)
      n = top;
    return n;
  }



double getBaseAvg() {
  return (
    L1BASE.rotation(deg) + L2BASE.rotation(deg) + L3BASE.rotation(deg)
   + R1BASE.rotation(deg) + R2BASE.rotation(deg) + R3BASE.rotation(deg)
   ) / 6;
}

double change = 2.125;// change distance cuz gear ratio

// Convert between inches and ticks of tracking wheels using dimensional analysis
double inchesToTicks(double inches) {
  return inches * (360 / TRACKING_CIRCUMFERENCE) * change;
}

double ticksToInches(double ticks) {
  return ticks * (TRACKING_CIRCUMFERENCE / 360) * change;
}


// SENSOR FUNCTIONS ///////////////////////////////////////////////////////
// return average motor encoder inches
double getTotalDistance() {
  return ticksToInches(
    L1BASE.rotation(deg) +
    L2BASE.rotation(deg) +
    L3BASE.rotation(deg) +
    R1BASE.rotation(deg) + 
    R2BASE.rotation(deg) + 
    R3BASE.rotation(deg)) / 6;
}

// reset motor encoders
void resetTotalDistance() {
  R1BASE.resetRotation();
  R2BASE.resetRotation();
  R3BASE.resetRotation();
  L1BASE.resetRotation();
  L2BASE.resetRotation();
  L3BASE.resetRotation();
}


// Obtain the current inertial sensor rotation relative to the starting rotation
double getRotationDeg() {
  return -INERTIAL.rotation(deg); // Important that this is negative
  // This makes positive angles CCW (left) and negative angles CW (right)
  // 0 degrees is on the positive x-axis
}

// Obtain the current inertial sensor rotation in radians
// A full rotation is 2 PI instead of 360
// Trig functions natively use radians
double getRotationRad() {
  return getRotationDeg() * toRadians;
}


// BASIC MOVEMENT ////////////////////////////

// Set the speeds of different sides of the base
void setLeftBase(double speed) {
  L1BASE.spin(fwd, speed, pct);
  L2BASE.spin(fwd, speed, pct);
  L3BASE.spin(fwd, speed, pct);
}

void setRightBase(double speed) {
  R1BASE.spin(fwd, speed, pct);
  R2BASE.spin(fwd, speed, pct);
  R3BASE.spin(fwd, speed, pct);
}

void setBase(double speed) {
  setLeftBase(speed);
  setRightBase(speed);
}

void stopBase() {
  L1BASE.stop(coast);
  L2BASE.stop(coast);
  L3BASE.stop(coast);
  R1BASE.stop(coast);
  R2BASE.stop(coast);
  R3BASE.stop(coast);
}


void holdBase() {
  L1BASE.stop(hold);
  L2BASE.stop(hold);
  L3BASE.stop(hold);
  R1BASE.stop(hold);
  R2BASE.stop(hold);
  R3BASE.stop(hold);
}


/*
void forwardConst(double inches, double speed) {
    resetTotalDistance();
    task::sleep(15);
    if (inches < 0) { speed *=-1; }
    setBase(speed);

    while(fabs(getTotalDistance()) < inches) {
      task::sleep(10);
    }

    stopBase();
}*/



void fwdConst(double amount, double speed, int timeout) {

  L1BASE.setTimeout(timeout, msec);
  L2BASE.setTimeout(timeout, msec);
  L3BASE.setTimeout(timeout, msec);
  R1BASE.setTimeout(timeout, msec);
  R2BASE.setTimeout(timeout, msec);
  R3BASE.setTimeout(timeout, msec);

  L1BASE.startRotateFor(amount, deg, speed, velocityUnits::pct);
  L2BASE.startRotateFor(amount, deg, speed, velocityUnits::pct);
  L3BASE.startRotateFor(amount, deg, speed, velocityUnits::pct);
  R1BASE.startRotateFor(amount, deg, speed, velocityUnits::pct);
  R2BASE.startRotateFor(amount, deg, speed, velocityUnits::pct);
  R3BASE.rotateFor(amount, deg, speed, velocityUnits::pct);
}


void revConst(double amount, double speed, int timeout) {

  L1BASE.setTimeout(timeout, msec);
  L2BASE.setTimeout(timeout, msec);
  L3BASE.setTimeout(timeout, msec);
  R1BASE.setTimeout(timeout, msec);
  R2BASE.setTimeout(timeout, msec);
  R3BASE.setTimeout(timeout, msec);

  L1BASE.startRotateFor(reverse, amount, deg, speed, velocityUnits::pct);
  L2BASE.startRotateFor(reverse, amount, deg, speed, velocityUnits::pct);
  L3BASE.startRotateFor(reverse, amount, deg, speed, velocityUnits::pct);
  R1BASE.startRotateFor(reverse, amount, deg, speed, velocityUnits::pct);
  R2BASE.startRotateFor(reverse, amount, deg, speed, velocityUnits::pct);
  R3BASE.rotateFor(reverse, amount, deg, speed, velocityUnits::pct);
}

// LEFT (CCW) IS POSITIVE
void turnR(double amount, double speed) {
  L1BASE.startRotateFor(amount, deg, speed, velocityUnits::pct);
  L2BASE.startRotateFor(amount, deg, speed, velocityUnits::pct);
  L3BASE.startRotateFor(amount, deg, speed, velocityUnits::pct);
  R1BASE.startRotateFor(reverse, amount, deg, speed, velocityUnits::pct);
  R2BASE.startRotateFor(reverse, amount, deg, speed, velocityUnits::pct);
  R3BASE.rotateFor(reverse, amount, deg, speed, velocityUnits::pct);
}

void turnL(double amount, double speed) {
  R1BASE.startRotateFor(amount, deg, speed, velocityUnits::pct);
  R2BASE.startRotateFor(amount, deg, speed, velocityUnits::pct);
  R3BASE.startRotateFor(amount, deg, speed, velocityUnits::pct);
  L1BASE.startRotateFor(reverse, amount, deg, speed, velocityUnits::pct);
  L2BASE.startRotateFor(reverse, amount, deg, speed, velocityUnits::pct);
  L3BASE.rotateFor(reverse, amount, deg, speed, velocityUnits::pct);
}


void fwdLerp(double targetInches, double startSpeed, double endSpeed) {
  resetTotalDistance();

  while(true) {
    double t = getTotalDistance()/targetInches; // ratio of distance travelled (starts at 0 and ends at 1)
    double speed = (startSpeed * t) + (endSpeed * (1-t)); // Gradually transition between startSpeed and endSpeed

    if (t >= 1) {
      break;
    }
    setLeftBase(speed);
    setRightBase(speed);
  }
  stopBase();
}

// make everything negative to go backward
void revLerp(double inches, double startSpeed, double endSpeed) {
  fwdLerp(-inches, -startSpeed, -endSpeed);
}



void turnLerp(double targetAngleRelative, double startSpeed, double endSpeed) {
  double targetAngleAbsolute = getRotationDeg() + targetAngleRelative;
  double initialError = targetAngleAbsolute - getRotationDeg();
  double error = initialError;

  while(true) {
    double t = error/initialError; // ratio of distance travelled (starts at 1 and ends at 0)
    double speed = (startSpeed * (1-t)) + (endSpeed * t); // Gradually transition between startSpeed and endSpeed

    if (t <= 0) {
      break;
    }
    setLeftBase(-speed);
    setRightBase(speed);
  }
  stopBase();
}


/*void lerpTurn(double relativeAngle) {
}*/


// Speed up and slow down gradually
// Use negative distance to go backwards
// More info: https://www.vexforum.com/t/advanced-pid-and-motion-profile-control/28400/3
void forwardInches(double inches, int maxSpeed) {
  resetTotalDistance();

  //MinSpeed : Ensures the movement overcomes friction.
  const double minSpeed = 3;      // Lowest speed the motors will go; Turning is more precise but jankier when lower.

  // Acceleration rates: changes rate of speed up/slow down
  // Gradually speeds up for the first half, slows down for second half
  const double accelRate = 100;    // Speed multiplier while speeding up (slope at start).
  const double deaccelRate = 6;  // Speed multiplier while slowing down (slope at end). Starts slowing down later when higher

  double targetDistance = inches;/*inchesToTicks(inches)*/;  // How far the robot should travel
  double targetRange = .3;                         // Distance from the desired distance the robot has to be to stop.
  double error = targetDistance;                  // Distance from the desired distance
  double speed;                                   // Actual speed value of the motors

  while (fabs(error) > targetRange) {

    error = targetDistance - getTotalDistance(); // Error = desired - actual

    // First half: accelerate to max. Second half, deccelerate to min
      // Speed is proportional to the error/target distance.
      // Since error is always decreasing, the speed proportion is inverted (subtracted from 1) for the first half of distance
      // travelled so speed increases proportionally instead of decreasing. 
      // fabs() (absolute value) is used because the direction is determined by the sign of the error, not of the speed.
    if (fabs(error) > fabs(targetDistance/2)) {
      speed = (1 - (fabs(error) / fabs(targetDistance))) * accelRate; // Speeds up as error decreases
    }  else {
      speed = (fabs(error) / fabs(targetDistance)) * deaccelRate;  // Slows down as error increases
    }
    speed = fabs(speed);
    speed *= maxSpeed;
    speed = keepInRange(speed, minSpeed, maxSpeed);

    // Go forward if positive error and backwards if negative error
    if (error < 0) {
      speed *= -1;
    }

    //if (signbit(speed) != signbit(maxSpeed)) {break;}

    setLeftBase(speed);
    setRightBase(speed);

    Brain.Screen.printAt(10, 150, "Target Dist: %.1f  ", targetDistance);
    Brain.Screen.printAt(10, 180, "Error:       %.1f      ", error);
    Brain.Screen.printAt(10, 200, "Speed:       %.1f  ", speed);
    task::sleep(10);
  }

  stopBase();
  task::sleep(15);
}




void forwardInchesTimed(double inches, int maxSpeed, int maxTimeMs) {
  resetTotalDistance();
  //maxSpeed = keepInRange(maxSpeed, 0, 100);

  //MinSpeed :  Ensures the movement overcomes friction.
  const double minSpeed = 3;      // Lowest speed the motors will go; Turning is more precise but jankier when lower.

  // Acceleration rates: changes rate of speed up/slow down
  // Gradually speeds up for the first half, slows down for second half
  const double accelRate = 100;    // Speed multiplier while speeding up (slope at start).
  const double deaccelRate = 4.5;  // Speed multiplier while slowing down (slope at end). Starts slowing down later when higher

  double targetDistance = inches;  // How far the robot should travel
  double targetRange = 0.3;                         // Distance from the desired distance the robot has to be to stop.
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
    if (fabs(error) > fabs(targetDistance/2)) {
      speed = (1 - (fabs(error) / fabs(targetDistance))) * accelRate; // Speeds up as error decreases
    } else {
      speed = (fabs(error) / fabs(targetDistance)) * deaccelRate;  // Slows down as error increases
    }
    speed = fabs(speed);
    speed *= maxSpeed;
    speed = keepInRange(speed, minSpeed, maxSpeed);

    // Go forward if positive error and backwards if negative error
    if (error < 0) {
      speed *= -1;
    }

    setLeftBase(speed);
    setRightBase(speed);
    
    Brain.Screen.printAt(10, 150, "Target Dist: %.1f  ", (targetDistance));
    Brain.Screen.printAt(10, 180,"Error:       %.1f     ", error);
    Brain.Screen.printAt(10, 200,"Speed:       %.1f  ", speed);

    if (speed > 0 && inches < 0) {break;}
    if (speed < 0 && inches > 0) {break;}

    task::sleep(10);
  }

  stopBase();
  task::sleep(15);
}


// Absolute turn in degrees
void gyroTurn(double targetAngle, int maxSpeed) {
  double initialAngle = getRotationDeg();
  //          <---------------------------|------------------->
  // Negative Degrees (Counterclockwise),    Positive Degrees (Clockwise)
  double relativeAngle = targetAngle - getRotationDeg();

  double targetRange = 1.0; // Distance from the desired angle that is allowed
  double error = relativeAngle; // Distance from the desired range
  double progress;              // Degrees the robot has turned already
  double minSpeed = 2; // Lowest speed the motors will go; Turning is generally more precise when lower, but slower.
  double deaccelRate = 1.35;//3; // 2.4

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

    controllerPrim.Screen.clearScreen();
    task::sleep(10);
    controllerPrim.Screen.setCursor(1, 1);
    task::sleep(10);
    controllerPrim.Screen.print("%f", getRotationDeg());

    Brain.Screen.printAt(10, 150, "Desired angle: %.1f, Relative angle: %.1f", degrees, relativeAngle);
    Brain.Screen.printAt(10, 180, "Speed: %.1f, Error: %.1f", speed, error);
    Brain.Screen.printAt(10, 200, "Gyro: %.1f", getRotationDeg());
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
      - Provides a tiny bit of speed at the end to overcome friction & weight of robot.
    D: Derivative
      - If you’re getting closer to where you want to be, slow down.
      - Smooths out any unwanted oscillations and prevent overshoot.

  PID is tuned by changing the values Kp, Ki, and Kd.
  They control how much each variable affects the speed output.
*/


void forwardPID(double targetInches, double maxSpeed, int msTimeout) {
    printf("fwd  %f \n", targetInches);
  // TUNE THESE
  double Kp = 4.75;
  double Ki = 0.005;
  double Kd = 2;

  double errorThreshold = 0.15; // Only exit loop when error is less than this
  //double derTheshold = 0.025; // Only exit loop when derivative is less than this
  int settleTimeThreshold = 125; // Only exit loop when within error threshold for this amount of milliseconds
  //double minSpeed = 0; // stay at least this fast to overcome friction. May be more reliable than using integral
  double integralPowerLimit = 5 / Ki; // how much speed the integral can contribute
  double integralActiveZone = 5; // How close to error the integral adds up
  
  resetTotalDistance();
  //resetDriveStraight();
  Brain.resetTimer();
  double integral;
  double lastError;
  int settleTimer = 0;
  double curMax = 5;
  bool continueLoop = true;

  while (continueLoop) {
    
    // ERROR: DISTANCE FROM TARGET. MOST SPEED COMES FROM HERE
    double error = targetInches - getTotalDistance(); // desired - actual

    // INTEGRAL: ACUMULATE SPEED TO GET FASTER AT END
    integral = integral + error;
    if (fabs(error) > integralActiveZone) { integral = 0; } // reset integral if outside range
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit); // keep integral within range

    // DERIVATIVE: SLOW DOWN TOWARDS END
    double derivative = error - lastError;
    lastError = error;

    double speed = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    speed = keepInRange(speed, -maxSpeed, maxSpeed);
    
    curMax += 4;
    speed = keepInRange(speed, -curMax, curMax);
    /*if (speed > 0 && speed < minSpeed)  { // go at least minspeed forward
      speed = minSpeed;
    } else if (speed > -minSpeed && speed < 0) { // go at least minspeed backward
      speed = -minSpeed;
    }*/
    setLeftBase(speed/* + correctionL*/);
    setRightBase(speed/* + correctionR*/);
    Brain.Screen.printAt(10, 180, "P: %.2f, I: %.2f, D: %.2f ", (Kp * error), (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(10, 200, "Dist: %.2f, Error: %.2f ", getTotalDistance(), error);
    vex::task::sleep(20);
    //graph(error);
    // Exit loop if within certain distance AND slow enough to not overshoot
    //if (fabs(error) < errorThreshold/* && fabs(derivative) <= derTheshold*/) { continueLoop = false; }

    if (fabs(error) < errorThreshold/* && fabs(derivative) <= derTheshold*/) {
      settleTimer += 20;
    } else {
      settleTimer = 0;
    }
    if (settleTimer > settleTimeThreshold) {
      continueLoop = false;
    }

    // Exit loop if run out of time. No timeout by default if not specified (set to -1).
    if (Brain.timer(vex::timeUnits::msec) > msTimeout && msTimeout != -1) { continueLoop = false; }
  }
  stopBase();
}



void forwardPIDStraight(double targetInches, double maxSpeed, double targetAngle, int msTimeout) {
    printf("fwd  %f \n", targetInches);
  // TUNE THESE
  double Kp = 6;
  double Ki = 0.001;
  double Kd = 1;

  double errorThreshold = 0.15; // Only exit loop when error is less than this
  //double derTheshold = 0.025; // Only exit loop when derivative is less than this
  int settleTimeThreshold = 125; // Only exit loop when within error threshold for this amount of milliseconds
  double minSpeed = 0; // stay at least this fast to overcome friction. May be more reliable than using integral
  double integralPowerLimit = 5 / Ki; // how much speed the integral can contribute
  double integralActiveZone = 5; // How close to error the integral adds up
  
  resetTotalDistance();
  //resetDriveStraight();
  Brain.resetTimer();
  double integral;
  double lastError;
  double settleTimer = 0;
  bool continueLoop = true;
  

  double correctionL = 0;
  double correctionR = 0;
  double angleError = 0;

  while (continueLoop) {
    
    // keep straight
    angleError = targetAngle - getRotationDeg();
    double cSpeed = angleError * 0.6;
    double maxCSpeed = 3;
    if (cSpeed > maxCSpeed) { cSpeed = maxCSpeed; }
    if (cSpeed < -maxCSpeed) { cSpeed = -maxCSpeed; }
    correctionL = -cSpeed;
    correctionR = cSpeed;


    // ERROR: DISTANCE FROM TARGET. MOST SPEED COMES FROM HERE
    double error = targetInches - getTotalDistance(); // desired - actual

    // INTEGRAL: ACUMULATE SPEED TO GET FASTER AT END
    integral = integral + error;
    if (fabs(error) > integralActiveZone) { integral = 0; } // reset integral if outside range
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit); // keep integral within range

    // DERIVATIVE: SLOW DOWN TOWARDS END
    double derivative = error - lastError;
    lastError = error;

    double speed = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    speed = keepInRange(speed, -maxSpeed, maxSpeed);
    if (speed > 0 && speed < minSpeed)  { // go at least minspeed forward
      speed = minSpeed;
    } else if (speed > -minSpeed && speed < 0) { // go at least minspeed backward
      speed = -minSpeed;
    }
    setLeftBase(speed + correctionL);
    setRightBase(speed + correctionR);
    Brain.Screen.printAt(10, 180, "P: %.2f, I: %.2f, D: %.2f ", (Kp * error), (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(10, 200, "Dist: %.2f, Error: %.2f", getTotalDistance(), error);
    vex::task::sleep(20);
    //graph(error);
    // Exit loop if within certain distance AND slow enough to not overshoot
    //if (fabs(error) < errorThreshold && fabs(derivative) <= derTheshold) { continueLoop = false; }
    if (fabs(error) < errorThreshold/* && fabs(derivative) <= derTheshold*/) {
      settleTimer += 20;
    } else {
      settleTimer = 0;
    }
    if (settleTimer > settleTimeThreshold) {
      continueLoop = false;
    }
    
    // Exit loop if run out of time. No timeout by default if not specified (set to -1).
    if (Brain.timer(vex::timeUnits::msec) > msTimeout && msTimeout != -1) { continueLoop = false; }
  }
  stopBase();
}



void turnPID(double targetDeg, double maxSpeed, int msTimeout) {
  
    printf("turn  %f \n", targetDeg);
  // TUNE THESE
  /*double Kp = 0.68;
  double Ki = 0.01;
  double Kd = 0.3;//2325;*/

  // 2-11
  // double Kp = 0.68;
  // double Ki = 0.01;
  // double Kd = 0.6;
  
  // ok KI tuning
  // double Kp = 0.4;
  // double Ki = 0.005;
  // double Kd = 0.0;

  
  /*double Kp = 0.6;
  double Ki = 0.0125;
  double Kd = 0.275;*/
  // ok tuning 
  /*double Kp = 0.6;
  double Ki = 0.009; // I helps steady state error by slowly increasing speed
  double Kd = 0.325;*/
  double Kp = 0.575;
  double Ki = 0.00415; // I helps steady state error by slowly increasing speed
  double Kd = 0.425;

  double errorThreshold = 0.5; // Only exit loop when error is less than this
  int settleTimeThreshold = 125; // Only exit loop when within error threshold for this amount of milliseconds
  double integralPowerLimit = 2 / Ki; // How much speed the integral can contribute. Divide by Ki since you  multiply by Ki later
  double integralActiveZone = 10; // How close to error the integral adds up
  
  Brain.resetTimer();
  double integral;
  double lastError;
  bool continueLoop = true;
  int settleTimer = 0;

  while (continueLoop) {
    // ERROR: DISTANCE FROM TARGET. MOST SPEED COMES FROM HERE
    double error = targetDeg - getRotationDeg(); // desired - actual

    // INTEGRAL: ACUMULATE SPEED TO OVERCOME FRICTION AT THE END
    integral = integral + error;
    if (fabs(error) > integralActiveZone || (signbit(error)!=signbit(lastError))) { integral = 0; } // keep integral 0 until the end so it doesnt grow out of control
    else if (errorThreshold < 0.75) {errorThreshold += 0.00175;} // if not getting there fast enough, increase the error needed to exit
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit); // keep integral limited so it doesnt go crazy

    // DERIVATIVE: SLOW DOWN TOWARDS END
    double derivative = error - lastError;
    lastError = error;

    // FINAL SPEED VALUE: WEIGHT EACH TERM AND ADD UP
    double speed = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    speed = keepInRange(speed, -maxSpeed, maxSpeed);
    if (fabs(error) > errorThreshold)
    {
      if (speed > 0 && speed < 1) speed = 1;
      if (speed < 0 && speed > -1) speed = -1;
    }
    setLeftBase(-speed);
    setRightBase(speed);
    Brain.Screen.printAt(10, 180, "P: %.2f I: %.2f D: %.2f ", (Kp * error), (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(10, 200, "Speed: %.2f Err: %.2f Setl: %d ", speed, error, settleTimer);
    vex::task::sleep(10);

    // Exit loop if within certain distance AND slow enough to not overshoot
    if (fabs(error) < errorThreshold/* && fabs(derivative) <= derTheshold*/) {
      settleTimer += 10;
    } else {
      settleTimer = 0;
    }
    if (settleTimer > settleTimeThreshold) {
      continueLoop = false;
    }
    // Exit loop if run out of time. No timeout by default if not specified (set to -1).
    //if (Brain.timer(vex::timeUnits::msec) > msTimeout && msTimeout != -1) { continueLoop = false; }
  }
  stopBase();
}


void turnPIDSmall(double targetDeg, double maxSpeed, int msTimeout) {
  // TUNE THESE
  double Kp = 0.6;
  double Ki = 0.001;
  double Kd = 0.375;

  double errorThreshold = 0.5; // Only exit loop when error is less than this
  int settleTimeThreshold = 125; // Only exit loop when within error threshold for this amount of milliseconds
  double integralPowerLimit = 2 / Ki; // How much speed the integral can contribute
  double integralActiveZone = 3; // How close to error the integral adds up
  
  Brain.resetTimer();
  double integral;
  double lastError;
  bool continueLoop = true;
  int settleTimer = 0;

  while (continueLoop) {
    // ERROR: DISTANCE FROM TARGET. MOST SPEED COMES FROM HERE
    double error = targetDeg - getRotationDeg(); // desired - actual

    // INTEGRAL: ACUMULATE SPEED TO OVERCOME FRICTION AT THE END
    integral = integral + error;
    if (fabs(error) > integralActiveZone || (signbit(error)!=signbit(lastError))) { integral = 0; } // keep integral 0 until the end so it doesnt grow out of control
    else if (errorThreshold < 1) {errorThreshold += 0.001;} // if not getting there fast enough, increase the error needed to exit
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit); // keep integral limited so it doesnt go crazy

    // DERIVATIVE: SLOW DOWN TOWARDS END
    double derivative = error - lastError;
    lastError = error;

    // FINAL SPEED VALUE: WEIGHT EACH TERM AND ADD UP
    
    double speed = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    speed = keepInRange(speed, -maxSpeed, maxSpeed);
    if (fabs(error) > errorThreshold)
    {
      if (speed > 0 && speed < 1) speed = 1;
      if (speed < 0 && speed > -1) speed = -1;
    }
    setLeftBase(-speed);
    setRightBase(speed);
    Brain.Screen.printAt(10, 180, "P: %.2f I: %.2f D: %.2f ", (Kp * error), (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(10, 200, "Speed: %.2f Err: %.2f Setl: %d ", speed, error, settleTimer);
    vex::task::sleep(10);

    // Exit loop if within certain distance AND slow enough to not overshoot
    if (fabs(error) < errorThreshold/* && fabs(derivative) <= derTheshold*/) {
      settleTimer += 10;
    } else {
      settleTimer = 0;
    }
    if (settleTimer > settleTimeThreshold) {
      continueLoop = false;
    }
    // Exit loop if run out of time. No timeout by default if not specified (set to -1).
    //if (Brain.timer(vex::timeUnits::msec) > msTimeout && msTimeout != -1) { continueLoop = false; }
  }
  stopBase();
}




void curvePID(double targetInches, double maxSpeed, double leftMult, int msTimeout) {
  // TUNE THESE
  double Kp = 3.5;
  double Ki = 0.0075;
  double Kd = 8;

  double errorThreshold = 0.175; // Only exit loop when error is less than this
  //double derTheshold = 0.01; // Only exit loop when derivative is less than this
  int settleTimeThreshold = 150; // Only exit loop when within error threshold for this amount of milliseconds
  double minSpeed = 0; // stay at least this fast to overcome friction. May be more reliable than using integral
  double integralPowerLimit = 5 / Ki; // how much speed the integral can contribute
  double integralActiveZone = 5; // How close to error the integral adds up
  
  resetTotalDistance();
  //resetDriveStraight();
  Brain.resetTimer();
  double integral;
  double lastError;
  int settleTimer = 0;
  bool continueLoop = true;

  while (continueLoop) {
    // ERROR: DISTANCE FROM TARGET. MOST SPEED COMES FROM HERE
    double error = targetInches - getTotalDistance(); // desired - actual

    // INTEGRAL: ACUMULATE SPEED TO GET FASTER AT END
    integral = integral + error;
    if (fabs(error) > integralActiveZone) { integral = 0; } // reset integral if outside range
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit); // keep integral within range

    // DERIVATIVE: SLOW DOWN TOWARDS END
    double derivative = error - lastError;
    lastError = error;

    double speed = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    speed = keepInRange(speed, -maxSpeed, maxSpeed);
    if (speed > 0 && speed < minSpeed)  { // go at least minspeed forward
      speed = minSpeed;
    } else if (speed > -minSpeed && speed < 0) { // go at least minspeed backward
      speed = -minSpeed;
    }
    setLeftBase(speed*leftMult/* + correctionL*/);
    setRightBase(speed/* + correctionR*/);
    Brain.Screen.printAt(10, 180, "P: %.2f, I: %.2f, D: %.2f ", (Kp * error), (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(10, 200, "Dist: %.2f, Error: %.2f ", getTotalDistance(), error);
    vex::task::sleep(10);

    // Exit loop if within certain distance AND slow enough to not overshoot
    //if (fabs(error) < errorThreshold/* && fabs(derivative) <= derTheshold*/) { continueLoop = false; }
        if (fabs(error) < errorThreshold/* && fabs(derivative) <= derTheshold*/) {
      settleTimer += 10;
    } else {
      settleTimer = 0;
    }
    if (settleTimer > settleTimeThreshold) {
      continueLoop = false;
    }
    // Exit loop if run out of time. No timeout by default if not specified (set to -1).
    if (Brain.timer(vex::timeUnits::msec) > msTimeout && msTimeout != -1) { continueLoop = false; }
  }
  stopBase();
}


/*
    curMax += rate;

    speed = keepInRange(speed, -curMax, curMax);
*/