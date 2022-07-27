/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\joshu                                            */
/*    Created:      Sun Jun 05 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;


// VARIABLES ////////////////////////////////

#define PI 3.14159265
const double toRadians = PI / 180.0; // multiply degrees by this
const double toDegrees = 180.0 / PI;

const double WHEEL_CIRCUMFERENCE = 3.75 * PI;

/*
const double TL = 4.875; // left tracking wheel perpendicular distance from center
const double TR = 4.875; // right tracking wheel perpendicular distance from center
const double TB = 4.875; // back tracking wheel perpendicular distance from center
*/

double lastForwardReading = 0;
double lastSidewaysReading = 0;
double lastRotation = 0;


double globalX = 0;
double globalY = 0;

double targetX = 0;
double targetY = 0;
float targetAngle = 0;


// DEVICES ///////////////////////////////////

motor LFBASE(PORT13, true); // for new bot port 15 && for old bot port 11
//motor LMBASE(PORT2); // Left mid
motor LBBASE(PORT18, true);

motor RFBASE(PORT1);       // true for old bot
//motor RMBASE(PORT14,true); // Right mid
motor RBBASE(PORT19);       // true for old bot



inertial INERTIAL(PORT14);

triport Triport(PORT22); // Get reference for three-wire ports on brain
encoder encoderL(Triport.A); // left tracking wheel
encoder encoderR(Triport.E); // right tracking wheel
encoder encoderB(Triport.C); // back tracking wheel. Testing to see if we need this to deal with drift


controller controllerPrim(controllerType::primary);



// MATH FUNCTIONS /////////////////////////////////////////////

// Convert between inches and revolutions of tracking wheels
// Revolutions are simpler than degrees for non-motor encoders
double inchesToRevs(double inches)
{
  return inches / WHEEL_CIRCUMFERENCE;
}

double revsToInches(double revs)
{
  return revs * WHEEL_CIRCUMFERENCE;
}

double keepInRange(double n, double bottom, double top) {
  if (n < bottom)
    n = bottom;
  if (n > top)
    n = top;
  return n;
}

// Keep an angle in -180 to 180 range
double keepAngleIn180(double a)
{
  while(a > 180)
    {
      a -= 360;
    }
    while(a < -180)
    {
      a += 360;
    }
    return a;
}


double distanceTo(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}



// screen is 480 x 272
void drawPoint(double x, double y)
{
    double tile_size = 30;
    double draw_pos_x = ((x / 24) * tile_size) + 105;
    double draw_pos_y = ((-y / 24) * tile_size) + 105;
    
    Brain.Screen.setFillColor(purple);
    Brain.Screen.setPenWidth(0);
    Brain.Screen.drawCircle(draw_pos_x, draw_pos_y, 4);
    Brain.Screen.setFillColor(color(10, 80, 30)); // green in rgb
}


// Convert where the screen is tapped to field coordinates
double screenToGlobalX(double screenX)
{
    double tile_size = 30;
    double globalX = ((screenX - 105) / tile_size) * 24;

    /*double draw_pos_x = ((x/24) * tile_size) + 105;
    double draw_pos_y = ((y/24) * tile_size) + 105;*/
    return globalX;
}

// Convert where the screen is tapped to field coordinates
double screenToGlobalY(double screenY)
{
    double tile_size = 30;
    double globalY = -((screenY - 105) / tile_size) * 24;

    return globalY;
}





// SENSOR FUNCTIONS ///////////////////////////////////////////////////////

double getForwardReading() {
  double revs = (encoderL.rotation(rev) + encoderR.rotation(rev)) / 2;
  return revsToInches(revs);
}

double getSidewaysReading() {
  //return ticksToInches(encoderB.rotation(deg));
return revsToInches(encoderB.rotation(rev));
}


double getDegrees()
{
  return -INERTIAL.rotation(deg); // Important that this is negative
  // Positive angles are CCW and negative angles are CW like on the unit circle
}


double getRadians()
{
  return getDegrees() * toRadians;
}





// MOVEMENT FUNCTIONS ////////////////////////////


void leftDrive(double power)
{
  LFBASE.spin(fwd, power, pct);
  //LMBASE.spin(fwd, power, pct);
  LBBASE.spin(fwd, power, pct);
}

void rightDrive(double power)
{
  RFBASE.spin(fwd, power, pct);
  //RMBASE.spin(fwd, power, pct);
  RBBASE.spin(fwd, power, pct);
}

void drive(double power)
{
  leftDrive(power);
  rightDrive(power);
}


void stopBase()
{
  LFBASE.stop(coast);
  //LMBASE.stop(coast);
  LBBASE.stop(coast);

  RFBASE.stop(coast);
  //RMBASE.stop(coast);
  RBBASE.stop(coast);
}




//float finalFwdSpeed = 15;

float errorFwd = 0;
float lastErrorFwd = 0;
float integralFwd = 0;
float derivativeFwd = 0;


//float finalTurnSpeed = 15;

float errorTurn = 0;
float lastErrorTurn = 0;
float integralTurn = 0;
float derivativeTurn = 0;



float fwdPID(double targetX, double targetY, double maxSpeed)
{
  float Kp = 0.3;
  float Ki = 0.02;
  float Kd = 0.075;
  float integralPowerLimit =
      40 / Ki;                   // little less than half power in pct (percent)
  float integralActiveZone = 15;
  float errorThreshold = 0.75; // Exit loop when error is less than this

  float speed = 0;

  errorFwd = distanceTo(globalX, globalY, targetX, targetY);

  if (fabs(errorFwd) > errorThreshold)
  {
    if (fabs(errorFwd) < integralActiveZone) {
      integralFwd = integralFwd + errorFwd;
    } else {
      integralFwd = 0;
    }
    integralFwd = keepInRange(integralFwd, -integralPowerLimit, integralPowerLimit);

    derivativeFwd = errorFwd - lastErrorFwd;

    speed = (Kp * errorFwd) + (Ki * integralFwd) + (Kd * derivativeFwd);
    speed =  keepInRange(speed, -maxSpeed, maxSpeed);
    //Brain.Screen.printAt(210, 120, "P: %.1f, I: %.1f, D: %.1f    ", (Kp * errorFwd) + (Ki * integralFwd) + (Kd * derivativeFwd));
  }
  else
  {
    integralFwd = 0;
    derivativeFwd = 0;
    speed = 0;
    //Brain.Screen.printAt(210, 120, "PID Fwd Completed             ");
  }
  lastErrorFwd = errorFwd;
  
  return speed;
}



float turnPID(double target, double maxSpeed)
{
  float Kp = 0.3;
  float Ki = 0.02;
  float Kd = 0.075;
  float integralPowerLimit =
      40 / Ki;                   // little less than half power in pct (percent)
  float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks
  float errorThreshold = 0.75; // Exit loop when error is less than this

  float speed = 0;


  errorTurn = target - getDegrees();

  if (fabs(errorTurn) > errorThreshold)
  {
    if (fabs(errorTurn) < integralActiveZone) {
      integralTurn = integralTurn + errorTurn;
    } else {
      integralTurn = 0;
    }
    integralTurn = keepInRange(integralTurn, -integralPowerLimit, integralPowerLimit);

    derivativeTurn = errorTurn - lastErrorTurn;

    speed = ((Kp * errorTurn) + (Ki * integralTurn) + (Kd * derivativeTurn));
    speed =  keepInRange(speed, -maxSpeed, maxSpeed);
    //Brain.Screen.printAt(210, 160, "P: %.1f, I: %.1f, D: %.1f    ", (Kp * errorTurn), (Ki * integralTurn), (Kd * derivativeTurn));
  }
  else
  {
    integralTurn = 0;
    derivativeTurn = 0;
    speed = 0;
    //Brain.Screen.printAt(210, 160, "PID Turn Completed             ");
  }
  lastErrorTurn = errorTurn;

  return speed;
}






// ODOMETRY /////////////////////////////////////////////////////////////////////////////

// Run one odometry calculation to update global position
// Run in a parallel task during auton
void updatePosition()
{
    // Get the current encoder distances and rotation 
    double currentForward = getForwardReading();
    double currentSideways = getSidewaysReading();
    double currentRotation = getDegrees();

    // Get the change in encoder values since last update
    double forwardChange = currentForward - lastForwardReading;
    double sidewaysChange = currentSideways - lastSidewaysReading;
                                                                  //double rotationChange = currentRotation - lastRotation;

    // Calculate global position change due to going forward/backward
    // For example, this is what the forward calculation looks like
    
    /*
                      new
                    position
                      |
                      V
                    / |
       Forward    /   |
       change   /     | change in y
              /       |
            /         |
          /           |
        /            _|
      /_____________|_|
      ^   change in x
      |
    current
    position

    */

    // Accumulate each tiny change in position to the global position

    // Using trig, we can obtain the x and y components from the distance moved and current rotation
    globalX += forwardChange * cos(currentRotation * toRadians);
    globalY += forwardChange * sin(currentRotation * toRadians);

    // Position change due to moving sideways
    // Even if the robot doesn't drive sideways it still might jerk
    globalX += sidewaysChange * sin(currentRotation * toRadians);
    globalY += sidewaysChange * cos(currentRotation * toRadians);

    // Update the old values for the next update
    lastForwardReading = currentForward;
    lastSidewaysReading = currentSideways;
    lastRotation = currentRotation;
}



// Keep rotation within 180 degrees of the reference (robot) so it doesnt turn too much
double angleWrap(double a, double referenceAngle)
{
  while(a > referenceAngle + 180) // Subtract 360 if angle is too large
  {
    a -= 360;
  }
  while(a <= referenceAngle - 180) // Add 360 if angle is too large
  {
    a += 360;
  }

  return a;
}


double getAngleToPosition(double x, double y)
{
  double relativeX = x - globalX; // error = desired - actual
  double relativeY = y - globalY;
  
  double currentAngle = getDegrees();

  // atan2 gives the angle to any position relative to the robot
  double angleToPosition = toDegrees * atan2(relativeY, relativeX);

  // Prevent the robot from targeting a rotation over 180 degrees from its current rotation.
  // If it's more than 180 it's faster to turn the other direction
  angleToPosition = angleWrap(angleToPosition, currentAngle);
  return angleToPosition;
}



void moveTo(double x, double y, double fwdSpeed, double turnSpeed)
{
  double finalFwdSpeed = 1;
  double finalTurnSpeed = 1;

  targetX = x;
  targetY = y;

  // Run while moving forward/turning
  while (finalFwdSpeed != 0 || finalTurnSpeed != 0)
  {
    finalFwdSpeed = fwdPID(targetX, targetY, fwdSpeed);
    finalTurnSpeed = turnPID(targetAngle, turnSpeed);

    // don't change angle when close so the robot doesn't run in circles
    // could get stuck if at a bad angle tho. maybe make a timeout or change the threshold (not too much)
    if (errorFwd > 1)
    {
      targetAngle = getAngleToPosition(targetX, targetY);
    }

    /*if (errorTurn > 180) // slow down fwd if need to turn a lot
    {
      finalFwdSpeed *= 0.75f;
    }*/

    leftDrive(finalFwdSpeed + finalTurnSpeed);
    rightDrive(finalFwdSpeed - finalTurnSpeed);
    Brain.Screen.printAt(210, 120, "target: (%.1f, %.1f) %.1f deg", targetX, targetY, targetAngle);
  }

  Brain.Screen.printAt(210, 120, "Done moving                    ");
}



// DISPLAY ///////////////////////////


// Draw the dashboard to visually display the robot's location and position
void draw()
{
    // Draw grid of the field
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(color(100, 100, 100));
    Brain.Screen.setPenWidth(2);

    int tile_size = 30;

    for (int x = 0; x < 6; x++)
    {
      for (int y = 0; y < 6; y++)
      {
        Brain.Screen.drawRectangle(x * tile_size + 15, y * tile_size + 15, tile_size, tile_size); // fill entire screen
      }
    }
    Brain.Screen.printAt(170, 100, "+x");
    Brain.Screen.printAt(20, 100, "-x");
    Brain.Screen.printAt(110, 30, "+y");
    Brain.Screen.printAt(110, 190, "-y");
    
    // Draw robot
    Brain.Screen.setFillColor(white);
    Brain.Screen.setPenWidth(0);
    double draw_pos_x = ((globalX/24) * tile_size) + 105;
    double draw_pos_y = ((-globalY/24) * tile_size) + 105; // make y negative because down is positive on the screen

     // Position of circle indicates the position found by odometry
    Brain.Screen.drawCircle(draw_pos_x, draw_pos_y, 6);
    Brain.Screen.setPenColor(red);
    Brain.Screen.setPenWidth(5);

    // Line indicates the rotation found by the gyro
    Brain.Screen.drawLine(draw_pos_x,
                          draw_pos_y,
                          draw_pos_x + cos(getDegrees() * toRadians) * 20,
                          draw_pos_y - sin(getDegrees() * toRadians) * 20); // make y negative because down is positive on the screen
    drawPoint(targetX, targetY);
}

void debug()
{
  
    Brain.Screen.setFillColor(color(10, 80, 30)); // green in rgb
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(210, 30, "Pos: (%.1f, %.1f)     ", globalX, globalY);
    Brain.Screen.printAt(210, 50, "Rot: %.1f deg      ", getDegrees());
    Brain.Screen.printAt(210, 70, "Encoders: (%.1f, %.1f)     ", getSidewaysReading(), getForwardReading());

    if (Brain.Screen.pressing())
    {
      targetX = screenToGlobalX(Brain.Screen.xPosition());
      targetY = screenToGlobalY(Brain.Screen.yPosition());
    }
}

int backgroundTasks()
{
  while (true)
  {
    updatePosition();
    debug();
    draw();
    task::sleep(15);
  }
  return 0;
}



int main()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  Brain.Screen.setFillColor(color(10, 80, 30)); // green in rgb
  Brain.Screen.setPenWidth(0);
  Brain.Screen.drawRectangle(0, 0, 480, 272); // fill entire screen


  while (INERTIAL.isCalibrating())
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(210, 50, "Rot: CALLIBRATING");
  }

  task a (backgroundTasks);

  moveTo(10, 10, 25, 20);

  while (1)
  {

    /*turnToPoint(10);

    Brain.Screen.printAt(210, 120, "Desired: (%.1f, %.1f)", targetX, targetY);

    updateFwdPID();
    updateTurnPID();*/

    /*double driveSpeed = 0;
    if (controllerPrim.ButtonUp.pressing())
    {
      driveSpeed = 10;
    }
    else if (controllerPrim.ButtonDown.pressing())
    {
      driveSpeed = -10;
    }

    leftDrive(driveSpeed + finalFwdPower + finalTurnPower);
    rightDrive(driveSpeed + finalFwdPower - finalTurnPower);*/



    /*if (controllerPrim.ButtonUp.pressing())
    {
      drive(10);
    }
    else if (controllerPrim.ButtonDown.pressing())
    {
      drive(-10);
    }
    else
    {
      stopBase();
    }*/

    task::sleep(15);
  }
}
