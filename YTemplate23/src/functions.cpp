#include "vex.h" // include vex files in this file

using namespace vex; // use vex keywords by default


void goTime(double numSeconds, double speed) {
  RBASE.spin(forward, speed, velocityUnits::pct);
  LBASE.spin(forward, speed, velocityUnits::pct);
  wait(numSeconds, seconds);

  // There's 3 brake types you can use to get different effects when breaking.
  // - coast glides to a stop (pretty sure coast is the default braketype if you don't put anything in the parentheses)
  // - brake stops kinda hard
  // - hold stops the motors hard
  LBASE.stop(brakeType::coast);
  RBASE.stop(brakeType::coast);
}

void goDistance(double distance, double speed) {
  RBASE.rotateFor(distance, rotationUnits::deg, speed, velocityUnits::pct);
  LBASE.rotateFor(distance, rotationUnits::deg, speed, velocityUnits::pct);
}


void turnDistanceL(double distance, double speed) {
  RBASE.rotateFor(distance, rotationUnits::deg, speed, velocityUnits::pct);
  LBASE.rotateFor(-distance, rotationUnits::deg, speed, velocityUnits::pct);
}

void turnDistanceR(double distance, double speed) {
  RBASE.rotateFor(-distance, rotationUnits::deg, speed, velocityUnits::pct);
  LBASE.rotateFor(distance, rotationUnits::deg, speed, velocityUnits::pct);
}


void driveStraight(double distance, double speed) {
  LBASE.resetRotation();

  while(LBASE.rotation(deg) < distance) {

    // keep straight
      double angleError = 0 - INERTIAL.rotation(deg); // find angle from 0
      double correctSpeed = angleError * 0.7;
      double maxCorrectSpeed = 4;
      // make sure correction speed is not too fast
      if (correctSpeed > maxCorrectSpeed) { correctSpeed = maxCorrectSpeed; }
      if (correctSpeed < -maxCorrectSpeed) { correctSpeed = -maxCorrectSpeed; }

      LBASE.spin(directionType::fwd, speed + correctSpeed, velocityUnits::pct);
      RBASE.spin(directionType::fwd, speed - correctSpeed, velocityUnits::pct);
  }

  LBASE.stop();
  RBASE.stop();

  // try to correct in case the loop doesn't end in the right place
  /*LBASE.spinToPosition(distance, deg, false);
  RBASE.spinToPosition(distance, deg);*/
}

void turnAngleR(double degrees, double speed) {
  while (INERTIAL.rotation() < degrees) {
    LBASE.spin(directionType::fwd, speed, velocityUnits::pct);
    RBASE.spin(directionType::rev, speed, velocityUnits::pct);
  }
  LBASE.stop();
  RBASE.stop();

  // how far off it is at the end
  double finalError = degrees - INERTIAL.rotation();
  INERTIAL.setRotation(-finalError, deg); // reset it so that the correct angle is always at 0
}


void turnAngleL(double degrees, double speed) {
  while (INERTIAL.rotation() > -degrees) {
    LBASE.spin(directionType::rev, speed, velocityUnits::pct);
    RBASE.spin(directionType::fwd, speed, velocityUnits::pct);
  }
  LBASE.stop();
  RBASE.stop();

  // how far off it is at the end
  double finalError = degrees - INERTIAL.rotation();
  // usually you'd set the rotation back to 0
  // but here it accounts for how off it was at the end
  INERTIAL.setRotation(-finalError , deg);
  
}


// CURVING: left and right motors have different speed.

void curveL(double leftDistance, double Lspeed, double Rspeed) {
  RBASE.spin(directionType::fwd, Rspeed, velocityUnits::pct); // start right side
  LBASE.rotateFor(leftDistance, rotationUnits::deg, Lspeed, velocityUnits::pct); // wait for left motors to reach leftDistance
  RBASE.stop(); // stop right side
}

void curveR(double rightDistance, double Lspeed, double Rspeed) {
  LBASE.spin(directionType::fwd, Lspeed, velocityUnits::pct);
  RBASE.rotateFor(rightDistance, rotationUnits::deg, Rspeed, velocityUnits::pct);
  LBASE.stop();
}


// Turn to an angle using a P loop for a certain amount of time
// It's hard to make it perfect so use drivestraight afterwards to fix the angle
void turnP(double desiredAngle, int numSeconds) { // CURRENTLY UNTESTED
  timer MyTimer;
  MyTimer.clear(); // Timer starts automatically for some reason

  while(MyTimer.time(seconds) < numSeconds) {

    // ERROR //////////////////////////////////////////////////////
    double angleError = desiredAngle - INERTIAL.rotation();
    // Error is how many degrees the robot still needs to turn.
    // Error is always desired value - actual value.
    // Positive error means turn right, negative error means turn left

    // TUNING CONSTANT /////////////////////////////////////////
    double kP = 0.5;
    // This number basically controls how bouncy the movement is.
    // kP is multiplied by the error to obtain the final speed.

    // SPEED CALCULATION ///////////////////////////////////////////
    double speed = angleError * kP;
    // Speed is proportional to error. That's what a P controller is.

    LBASE.spin(forward, speed, pct);
    RBASE.spin(reverse, speed, pct);
  }
  LBASE.stop();
  RBASE.stop();
}