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

/*
// new version (7/15)
double lastEncoderL = 0;
double lastEncoderR = 0;
*/

double globalXPos = 0;
double globalYPos = 0;

double desiredX = 0;
double desiredY = 0;


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



float targetAngle = 0;
float maxPower = 15;
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.0;

float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

float integralPowerLimit =
    40 / Ki;                   // little less than half power in pct (percent)
float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks

float exitThreshold = 0.75; // Exit loop when error is less than this


void updatePID()
{
  error = targetAngle - getDegrees();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;

    float finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    finalPower =  keepInRange(finalPower, -maxPower, maxPower);

    leftDrive(-finalPower);
    rightDrive(finalPower); 
    Brain.Screen.printAt(210, 120, "P: %.2f, I: %.2f, D: %.2f", (Kp * error), (Ki * integral), (Kd * derivative));

    vex::task::sleep(40);
}



// Simple constant speed function for testing (smooth speed added later)
void turnToPoint(double turnSpeed)
{
  
  drawPoint(desiredX, desiredY);

  double relativeX = desiredX - globalXPos; // error = desired - actual
  double relativeY = desiredY - globalYPos;
  double angleToPosition;


  //while (true)
  {
    double speedL = 0;
    double speedR = 0;

    // Avoid dividing by zero when doing atan
    /*if (relativeX == 0)
    {
      angleToPosition = 90;
    }
    else
    {
      angleToPosition = toDegrees * atan(relativeY/relativeX); // get angle from opposite/adjacent ratio
    }*/

    double currentAngle = getDegrees();
    // Keep currentAngle within -180 to 180
    //currentAngle = keepAngleIn180(currentAngle);

    // atan2 gives the angle to any position relative to the robot
    angleToPosition = toDegrees * atan2(relativeY, relativeX);

    // Wraparound angle so that it is the closest to the current rotation

    // The angles go from -180 to 180
    // If the robot is at 179 and wants to go to -179, it would go all the way around instead of going 2 degrees the other way
    // Thi

    // Prevents the robot from going all the way around if  it is closer to go the other way
    double closestAngle = angleToPosition;
    while(closestAngle > currentAngle + 180)
    {
      // Adding/subtracting 360 doesn't change the angle, the robot just needs to rotate a different amount to get there
      closestAngle -= 360;
    }
    while(closestAngle < currentAngle - 180)
    {
      closestAngle += 360;
    }

    // https://stackoverflow.com/questions/28909231/how-to-find-the-nearest-angle-from-0-to-180-and-180-to-0
    
    // get the closest rotation of any revolution (since + or - 360 is the same angle)
    /*
    
    double error = angleToPosition - currentAngle; // how far off from angle (desired - actual)

    double closestError = fmin(fabs(error), 360 - fabs(error));
    if (closestError < error)
    {
      error = closestError;
    }

    targetAngle = error + currentAngle;*/

    
    // atan2 only seems to return angles within 0-360, but it's possible an angle outside that range is closer
    // compare the angle to the same angle but 1 revolution behind or ahead to get the closest  angle

    /*int revAhead = angleToPosition + 360;
    int revBehind = angleToPosition - 360;
    
    //  Brain.Screen.printAt(230, 140, "0: %d A: %d B: %d         ", (int)round(angleToPosition), (int)round(revAhead), (int)round(revBehind));
    if (fabs(revAhead - currentAngle) < fabs(angleToPosition - currentAngle))
    {
      angleToPosition = revAhead;
      Brain.Screen.printAt(230, 160, "+");
    }
    else if (fabs(revBehind - currentAngle) < fabs(angleToPosition - currentAngle))
    {
      angleToPosition = revBehind;
      Brain.Screen.printAt(230, 160, "-");
    }
    else
    {
      
      Brain.Screen.printAt(230, 160, "0");
    }*/

    targetAngle = closestAngle;

    //Brain.Screen.printAt(210, 100, "angleToPosition: %.1f   ", angleToPosition);

    Brain.Screen.printAt(210, 100, "closestAngle: %.1f   ", closestAngle);
    
    // distance from the targetAngle


                    /*double positionError =  sqrt(pow(desiredX - globalXPos, 2) + pow(desiredY - globalYPos, 2));
                    
                    if (positionError > 5) // go back if too far forward
                    {
                      RSpeed = moveSpeed;
                      LSpeed = moveSpeed;
                    }
                    else if (positionError < -5) // go forward if too far back
                    {
                      RSpeed = -moveSpeed;
                      LSpeed = -moveSpeed;
                    }
                */
    /*double angleError = angleToPosition - getDegrees(); // how far off from angleToPosition the robot is
    if (angleError > 1) // turn left if rotation too far left
    {
      speedR += turnSpeed;
      speedL -= turnSpeed;
    }
    else if (angleError < -1) // turn right if rotation too far left 
    {
      speedR -= turnSpeed;
      speedL += turnSpeed;
    }

    leftDrive(speedL);
    rightDrive(speedR);*/
    
    //updatePID();
  }
}




/*
void turnPID(float targetAngle, float maxPower) {

  float Kp = 0.48;
  float Ki = 0.005;
  float Kd = 0.499;
  float change = 1.0;

  float error = (targetAngle * change) - getDegrees();
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

  Brain.resetTimer();

  while (fabs(error) > exitThreshold) {
    error = (targetAngle * change) - getDegrees();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;

    finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    finalPower =  keepInRange(finalPower, -maxPower, maxPower);

    leftDrive(-finalPower);
    rightDrive(finalPower); 
    Brain.Screen.printAt(210, 120, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
                         (Ki * integral), (Kd * derivative));

    vex::task::sleep(40);
  }
  stopBase();
}*/





/*
  // New way that accounts for position of tracking wheels
  // https://www.youtube.com/watch?v=qqODIdvSGac&t=417s
  void updatePosition2()
  {
    //double currentEncoderX = getEncoderX();
    double currentEncoderY = getEncoderY();

    double radians = getRadians();




    double currentEncoderYR = getEncoderYR();
    double YRChange = currentEncoderYR - lastEncoderYR; // Calculate change in right Y encoder as an arc
    double centerYChange = (YRChange / radians) + TR; // Calculate center arc length (right arc + right offset from center)

    double currentEncoderX = getEncoderX();
    double xChange = currentEncoderX - lastEncoderX; // Calculate change in x encoder as arc
    double centerXChange = (xChange / radians) + TS;// Calculate center arc length (back arc + back offset from center)


    // Multiply x and y by this to make the angle correct
    double angleMultiplier = 2 * sin(radians / 2);
    globalYPos += angleMultiplier * centerYChange;
    globalXPos += angleMultiplier * centerXChange;

    //lastEncoderYL = currentEncoderYL;
    lastEncoderYR = currentEncoderYR;
    lastEncoderX = currentEncoderX;
  }
*/


// ART ///////////////////////////

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
      desiredX = screenToGlobalX(Brain.Screen.xPosition());
      desiredY = screenToGlobalY(Brain.Screen.yPosition());
    }
    turnToPoint(10);

    Brain.Screen.printAt(210, 120, "Desired: (%.1f, %.1f)", desiredX, desiredY);



    error = targetAngle - getDegrees();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;

    float turnPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    turnPower =  keepInRange(turnPower, -maxPower, maxPower);

    Brain.Screen.printAt(210, 140, "P: %.2f, I: %.2f, D: %.2f", (Kp * error), (Ki * integral), (Kd * derivative));



    double driveSpeed = 0;
    if (controllerPrim.ButtonUp.pressing())
    {
      driveSpeed = 10;
    }
    else if (controllerPrim.ButtonDown.pressing())
    {
      driveSpeed = -10;
    }

    leftDrive(driveSpeed + turnPower);
    rightDrive(driveSpeed - turnPower);
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
