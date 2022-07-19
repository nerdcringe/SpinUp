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

const double TL = 4.875; // left tracking wheel perpendicular distance from center
const double TR = 4.875; // right tracking wheel perpendicular distance from center
const double TB = 4.875; // back tracking wheel perpendicular distance from center


double lastForwardReading = 0;
double lastSidewaysReading = 0;
double lastRotation = 0;


double globalXPos = 0;
double globalYPos = 0;

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
};



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


float maxTurnPower = 20;
float finalTurnPower = 15;
float Kp = 0.3;
float Ki = 0.02;
float Kd = 0.075;

float turnError = 0;
float lastTurnError = 0;
float turnIntegral = 0;
float turnDerivative = 0;

float integralPowerLimit =
    40 / Ki;                   // little less than half power in pct (percent)
float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks

float turnErrorThreshold = 0.75; // Exit loop when error is less than this


void updateTurnPID()
{

  turnError = targetAngle - getDegrees();

  if (fabs(turnError) > turnErrorThreshold)
  {
    if (fabs(turnError) < integralActiveZone) {
      turnIntegral = turnIntegral + turnError;
    } else {
      turnIntegral = 0;
    }
    turnIntegral = keepInRange(turnIntegral, -integralPowerLimit, integralPowerLimit);

    turnDerivative = turnError - lastTurnError;

    finalTurnPower = ((Kp * turnError) + (Ki * turnIntegral) + (Kd * turnDerivative));
    finalTurnPower =  keepInRange(finalTurnPower, -maxTurnPower, maxTurnPower);
    Brain.Screen.printAt(210, 120, "P: %.1f, I: %.1f, D: %.1f", (Kp * turnError), (Ki * turnIntegral), (Kd * turnDerivative));
  }
  else
  {
    turnIntegral = 0;
    turnDerivative = 0;
    finalTurnPower = 0;
  Brain.Screen.printAt(210, 120, "PID Turn Completed             ");
  }
  lastTurnError = turnError;
  

  vex::task::sleep(40);
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
    globalXPos += forwardChange * cos(currentRotation * toRadians);
    globalYPos += forwardChange * sin(currentRotation * toRadians);

    // Position change due to moving sideways
    // Even if the robot doesn't drive sideways it still might jerk
    globalXPos += sidewaysChange * sin(currentRotation * toRadians);
    globalYPos += sidewaysChange * cos(currentRotation * toRadians);

    // Update the old values for the next update
    lastForwardReading = currentForward;
    lastSidewaysReading = currentSideways;
    lastRotation = currentRotation;
}






// Simple constant speed function for testing (smooth speed added later)
void turnToPoint(double turnSpeed)
{
  
  drawPoint(targetX, targetY);

  double relativeX = targetX - globalXPos; // error = desired - actual
  double relativeY = targetY - globalYPos;
  double angleToPosition;


  //while (true)
  {

    double currentAngle = getDegrees();

    // atan2 gives the angle to any position relative to the robot
    angleToPosition = toDegrees * atan2(relativeY, relativeX);

    // Prevent the robot from targeting a rotation over 180 degrees from its current rotation.
    // If it's more than 180 it's faster to turn the other direction
    double closestAngle = angleToPosition;
    while(closestAngle > currentAngle + 180) // Subtract 360 if angle is too large
    {
      closestAngle -= 360;
    }
    while(closestAngle <= currentAngle - 180) // Add 360 if angle is too large
    {
      closestAngle += 360;
    }
    targetAngle = closestAngle;

    Brain.Screen.printAt(210, 100, "closestAngle: %.1f   ", closestAngle);
  }
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
    
    // Draw robot
    Brain.Screen.setFillColor(white);
    Brain.Screen.setPenWidth(0);

    double draw_pos_x = ((globalXPos/24) * tile_size) + 105;
    double draw_pos_y = ((-globalYPos/24) * tile_size) + 105;

     // Position of circle indicates the position found by odometry
    Brain.Screen.drawCircle(draw_pos_x, draw_pos_y, 6);
    Brain.Screen.setPenColor(red);
    Brain.Screen.setPenWidth(5);

    // Line indicates the rotation found by the gyro
    Brain.Screen.drawLine(draw_pos_x,
                          draw_pos_y,
                          draw_pos_x + cos(getDegrees() * toRadians) * 20,
                          draw_pos_y - sin(getDegrees() * toRadians) * 20);
                          // make sine for y-position negative because (0, 0) on screen is top left, so bigger # is lower on screen
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


  while (1)
  {
    updatePosition();

    draw();

    Brain.Screen.setFillColor(color(10, 80, 30)); // green in rgb
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(210, 30, "Pos: (%.1f, %.1f)     ", globalXPos, globalYPos);
    Brain.Screen.printAt(210, 50, "Rot: %.1f deg      ", getDegrees());
    Brain.Screen.printAt(210, 70, "Encoders: (%.1f, %.1f)     ", getSidewaysReading(), getForwardReading());

    if (Brain.Screen.pressing())
    {
      targetX = screenToGlobalX(Brain.Screen.xPosition());
      targetY = screenToGlobalY(Brain.Screen.yPosition());
    }
    turnToPoint(10);

    Brain.Screen.printAt(210, 120, "Desired: (%.1f, %.1f)", targetX, targetY);



    /*error = targetAngle - getDegrees();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;

    float turnPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    turnPower =  keepInRange(turnPower, -maxTurnPower, maxTurnPower);

    Brain.Screen.printAt(210, 140, "P: %.2f, I: %.2f, D: %.2f", (Kp * error), (Ki * integral), (Kd * derivative));*/

    
    updateTurnPID();


    double driveSpeed = 0;
    if (controllerPrim.ButtonUp.pressing())
    {
      driveSpeed = 10;
    }
    else if (controllerPrim.ButtonDown.pressing())
    {
      driveSpeed = -10;
    }

    leftDrive(driveSpeed + finalTurnPower);
    rightDrive(driveSpeed - finalTurnPower);
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
