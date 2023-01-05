#include "vex.h"
using namespace vex;

// CONSTANTS //
#define PI 3.14159265
const double toRadians = PI / 180.0; // multiply degrees by this to convert to radians
const double toDegrees = 180.0 / PI; // multiply radians by this to convert to degrees

//const double WHEEL_CIRCUMFERENCE = 3.25 * PI; // Circumference of powered wheels (diameter * PI)
const double TRACKING_CIRCUMFERENCE = 2.75 * PI; // Circumference of tracking wheels (diameter * PI)


double degree = 0.0;
double KPC = 50.0;
double KDC = 0.12;
double correctionL = 0;
double correctionR = 0;

// MATH FUNCTIONS /////////////////////////////////////////////

// Return the number clamped between two numbers
double keepInRange(double n, double bottom, double top) {
  if (n < bottom)
    n = bottom;
  if (n > top)
    n = top;
  return n;
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


void forwardConst(double inches, double speed) {
    resetTotalDistance();
    task::sleep(15);
    if (inches < 0) { speed *=-1; }
    setBase(speed);

    while(fabs(getTotalDistance()) < inches) {
      task::sleep(10);
    }

    stopBase();
}


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
    setLeftBase(speed);
    setRightBase(speed);

    Brain.Screen.printAt(210, 150, "Target Dist: %.1f  ", targetDistance);
    Brain.Screen.printAt(210, 170, "Error:       %.1f      ", error);
    Brain.Screen.printAt(210, 190, "Speed:       %.1f  ", speed);
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
    
    Brain.Screen.printAt(210, 150, "Target Dist: %.1f  ", (targetDistance));
    Brain.Screen.printAt(210, 170,"Error:       %.1f     ", error);
    Brain.Screen.printAt(210, 190,"Speed:       %.1f  ", speed);
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

    Brain.Screen.printAt(210, 150, "Desired angle: %.1f, Relative angle: %.1f", degrees, relativeAngle);
    Brain.Screen.printAt(210, 170, "Speed: %.1f, Error: %.1f", speed, error);
    Brain.Screen.printAt(210, 190, "Gyro: %.1f", getRotationDeg());
    task::sleep(10);
  }

  stopBase();
  task::sleep(15);
}



double lastGraphX = 0;
double lastGraphY = 0;
// Display the graph of a variable over time
void graph(double value, color graphColor=color::black) {
  double x = timer::system()/30;
  double y = (value) * 8;
  // scale height so it is visible but also fits on the screen


  // draw y = 0 line
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setPenColor(color(255, 255, 255));
  Brain.Screen.drawLine(0, 136, 480, 136);

  // Draw data
  Brain.Screen.setPenWidth(0);
  Brain.Screen.setFillColor(graphColor);
  //Brain.Screen.drawLine(lastGraphX, -lastGraphY + 136, x, -y + 136);
  Brain.Screen.drawCircle(x, -lastGraphY + 136, 1);
  
  Brain.Screen.setPenColor(color(255, 255, 255));
  lastGraphX = x;
  lastGraphY = y;
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
      - Makes sure speed is fast enough to overcome friction & weight of robot.
    D: Derivative
      - If you’re getting closer to where you want to be, slow down.
      - Smooths out any unwanted oscillations and prevent overshoot.

  PID is tuned by changing the values Kp, Ki, and Kd.
  They control how much each variable affects the speed output.
*/

void forwardPID(double targetInches, double maxSpeed, int msTimeout) {
  // TUNE THESE
  double Kp = 9; 
  double Ki = 0.00;
  double Kd = 4;

  double errorThreshold = 0.125; // Only exit loop when error is less than this
  double derTheshold = 0.005; // Only exit loop when derivative is less than this
  double minSpeed = 1; // stay at least this fast to overcome friction. May be more reliable than using integral
  double integralPowerLimit = 5 / Ki; // how much speed the integral can contribute
  double integralActiveZone = 5; // How close to error the integral adds up
  
  resetTotalDistance();
  //resetDriveStraight();
  Brain.resetTimer();
  double integral;
  double lastError;
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
    setLeftBase(speed/* + correctionL*/);
    setRightBase(speed/* + correctionR*/);
    Brain.Screen.printAt(210, 170, "P: %.2f, I: %.2f, D: %.2f", (Kp * error), (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(210, 190, "Dist: %.2f, Error: %.2f", getTotalDistance(), error);
    vex::task::sleep(10);
    //graph(error);
    // Exit loop if within certain distance AND slow enough to not overshoot
    if (fabs(error) < errorThreshold && fabs(derivative) <= derTheshold) { continueLoop = false; }
    // Exit loop if run out of time. No timeout by default if not specified (set to -1).
    if (Brain.timer(vex::timeUnits::msec) > msTimeout && msTimeout != -1) { continueLoop = false; }
  }
  stopBase();
}



void turnPID(double targetDeg, double maxSpeed, int msTimeout) {
  // TUNE THESE
  /*double Kp = 0.3;
  double Ki = 0.000;
  double Kd = 0.1;*/
  // OK
  double Kp = 0.5925;
  double Ki = 0.000;
  double Kd = 0.2325;
  
  /* // wikipedia method
  double Kp = 0.9;
  double Ki = 0.000;
  double Kd = 0.2;*/
  // CHANGE UP WORLDS
  /*float Kp =  0.5105;   //getting to target
    float Ki =  0.001; // increases speed (builds up over time) before: 0.008
    float Kd =  0.02;    //slow down at end*/

  double errorThreshold = 0.3; // Only exit loop when error is less than this
  double derTheshold = 0.15; // Only exit loop when derivative is less than this
  double minSpeed = 1.25; // stay at least this fast to overcome friction. May be more reliable than using integral
  double integralPowerLimit = 3 / Ki; // How much speed the integral can contribute
  double integralActiveZone = 10; // How close to error the integral adds up
  
  Brain.resetTimer();
  double integral;
  double lastError;
  bool continueLoop = true;

  while (continueLoop) {
    // ERROR: DISTANCE FROM TARGET. MOST SPEED COMES FROM HERE
    double error = targetDeg - getRotationDeg(); // desired - actual

    // INTEGRAL: ACUMULATE SPEED TO OVERCOME FRICTION AT THE END
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
    setLeftBase(-speed);
    setRightBase(speed);
    Brain.Screen.printAt(210, 170, "P: %.2f, I: %.2f, D: %.2f", (Kp * error), (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(210, 190, "Speed: %.2f, Error: %.2f", speed, error);
    vex::task::sleep(10);
    //graph(L1BASE.velocity(pct));
    /*graph(error, color::red);
    graph(integral, color::white);
    graph(derivative, color::blue);
    graph(speed);*/

    // Exit loop if within certain distance AND slow enough to not overshoot
    if (fabs(error) < errorThreshold && fabs(derivative) <= derTheshold) { continueLoop = false; }
    // Exit loop if run out of time. No timeout by default if not specified (set to -1).
    //if (Brain.timer(vex::timeUnits::msec) > msTimeout && msTimeout != -1) { continueLoop = false; }
  }
  stopBase();
}


void turnPIDDist(double targetInches, double maxSpeed, int msTimeout) {
  // TUNE THESE
  double Kp = 9; 
  double Ki = 0.00;
  double Kd = 4;

  double errorThreshold = 0.125; // Only exit loop when error is less than this
  double derTheshold = 0.005; // Only exit loop when derivative is less than this
  double minSpeed = 1; // stay at least this fast to overcome friction. May be more reliable than using integral
  double integralPowerLimit = 5 / Ki; // how much speed the integral can contribute
  double integralActiveZone = 5; // How close to error the integral adds up
  
  resetTotalDistance();
  //resetDriveStraight();
  Brain.resetTimer();
  double integral;
  double lastError;
  bool continueLoop = true;

  while (continueLoop) {
    // ERROR: DISTANCE FROM TARGET. MOST SPEED COMES FROM HERE
    double error = targetInches - ticksToInches(R1BASE.rotation(deg)+R2BASE.rotation(deg)+R3BASE.rotation(deg))/3; // desired - actual

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
    setLeftBase(-speed/* + correctionL*/);
    setRightBase(speed/* + correctionR*/);
    Brain.Screen.printAt(210, 170, "P: %.2f, I: %.2f, D: %.2f", (Kp * error), (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(210, 190, "Dist: %.2f, Error: %.2f", getTotalDistance(), error);
    vex::task::sleep(10);
    //graph(error);
    // Exit loop if within certain distance AND slow enough to not overshoot
    if (fabs(error) < errorThreshold && fabs(derivative) <= derTheshold) { continueLoop = false; }
    // Exit loop if run out of time. No timeout by default if not specified (set to -1).
    if (Brain.timer(vex::timeUnits::msec) > msTimeout && msTimeout != -1) { continueLoop = false; }
  }
  stopBase();
}





void forwardPIDGradual(double targetInches, double maxSpeed, int msTimeout) {
  // TUNE THESE
  double Kp = 9; 
  double Ki = 0.00;
  double Kd = 4;

  double errorThreshold = 0.125; // Only exit loop when error is less than this
  double derTheshold = 0.005; // Only exit loop when derivative is less than this
  double minSpeed = 1; // stay at least this fast to overcome friction. May be more reliable than using integral
  double integralPowerLimit = 5 / Ki; // how much speed the integral can contribute
  double integralActiveZone = 5; // How close to error the integral adds up
  
  resetTotalDistance();
  //resetDriveStraight();
  Brain.resetTimer();
  double integral;
  double lastError;
  bool continueLoop = true;

  double curMax = 2.5;

  while (continueLoop) {
    curMax += 0.75;

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
    speed = keepInRange(speed, -curMax, curMax);
    speed = keepInRange(speed, -maxSpeed, maxSpeed);
    if (speed > 0 && speed < minSpeed)  { // go at least minspeed forward
      speed = minSpeed;
    } else if (speed > -minSpeed && speed < 0) { // go at least minspeed backward
      speed = -minSpeed;
    }
    setLeftBase(speed/* + correctionL*/);
    setRightBase(speed/* + correctionR*/);
    Brain.Screen.printAt(210, 170, "P: %.2f, I: %.2f, D: %.2f", (Kp * error), (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(210, 190, "Dist: %.2f, Error: %.2f", getTotalDistance(), error);
    vex::task::sleep(10);
    //graph(error);
    // Exit loop if within certain distance AND slow enough to not overshoot
    if (fabs(error) < errorThreshold && fabs(derivative) <= derTheshold) { continueLoop = false; }
    // Exit loop if run out of time. No timeout by default if not specified (set to -1).
    if (Brain.timer(vex::timeUnits::msec) > msTimeout && msTimeout != -1) { continueLoop = false; }
  }
  stopBase();
}

