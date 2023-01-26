# VEX Team 1575Y Code - Spin Up 2022-23

This year for Spin-Up we wanted to prioritize accuracy because we're launching discs. Precise positioning is important for hitting the goals. We worked on a position tracking system, called odometry, for more accurate movements. Odometry allows the robot to move based on a system of x-y coordinates. Odometry also makes curved paths more accurate.


# Code files

We organized the code into several files to make it more readable. Functions.cpp is for general functions, odom.cpp has odometry-specific functions, and main.cpp has the autons and driver control.

### Using this code
You can add functions.cpp to your code without odom.cpp, but odom.cpp requires functions.cpp. Be sure to put the corresponding .h file into the includes folder in VEXcode. Also add the following line to vex.h:
```
#include "functions.h"
#include "odom.h"
```

This will include the functions definitions from the header file into your code. Without this line, there is no way to access functions from other files in main.cpp.
 
## Functions.cpp

General functions for movement, sensors, and pneumatics

#### forwardPID 

- Parameters: double *distance* (inches), double *maxSpeed* (percent), int *timeoutMillis* (milliseconds)

- Description: Move forward/backwards for a certain distance using PID algorithm. Limit the output speed of the PID to a certain maximum speed. Exit if timeoutMillis is specified and the timer runs out.

 

#### turnPID 

- Parameters: double *angle* (degrees), double *maxSpeed* (percent) , int *timeoutMillis* (milliseconds)

- Description: Turn to a certain angle relative to the starting angle of the robot based on a PID algorithm. Limit the output speed of the PID to a certain maximum speed.  Exit if timeoutMillis is specified and the timer runs out.

 

#### forwardInches 

- Parameters: double *distance* (inches), double *maxSpeed* (percent), int *timeoutMillis* (milliseconds) 

- Description: Move forward/backwards for a certain distance at a speed that ramps up, maxes out and slows back down at the end. Use this for smaller distances that PID cannot handle well. Exit after a certain time (timeoutMs). 


#### fwdConst
- Parameters: double *distance* (inches), double *maxSpeed* (percent), int *timeoutMillis* (milliseconds) 

- Description: Move forward/backwards for a certain distance at a constant speed. Use this for smaller distances that PID cannot handle well. Exit after a certain time (timeoutMs).
 

## Odom.cpp
Odometry-specific code for moving to a specific point

#### updatePosition
- Description: Update the current position. Put this inside the while loop in int main to make this always run in the background.

#### setTarget 

- Parameters: double *x* (inches), double *y* (inches) 

- Description: Set the desired target position. Use this before moving to a target. 

 

#### turnToTarget 

- Parameters: double *turnSpeed* (percent) 

- Description: Turn to face the target position on the field. 

 
 
#### moveToTarget 

- Parameters: double *forwardSpeed* (percent), double *turnSpeed* (percent) 

- Description: Turn to the target position while moving forward to eventually stop on the target. 

 

#### moveToTargetRev 

- Parameters: double *backwardSpeed* (percent), double *turnSpeed* (percent) 

- Description: Turn to the target position while moving backward to eventually stop on the target. 

 

#### passTarget 

- Parameters: double *forwardSpeed* (percent), double *turnSpeed* (percent) 

- Description: Turn to the target position while moving forward to pass by the target. The robot will not slow down after it passes the target, so use this for intermediate movements. 

 

#### passTargetRev 

- Parameters: double *forwardSpeed* (percent), double *turnSpeed* (percent) 

- Description: Turn to the target position while moving backward to pass by the target. The robot will not slow down after it passes the target, so use this for intermediate movements. 

 
# Guide

## PID Controllers

A PID controller is an algorithm we use to move motors precisely. The default motor functions VexCode provides aren't very precise. PID allows you to customize the way a motor moves to make it more accurate for its use case. We use PID to move straight and turn. It can also be used for moving lifts, maintaining a constant flywheel speed, and more.

In PID, the distance or angle from a target is called *error*. The goal of PID is to reduce error as precisely as possible. Error is calculated with the formula: desired value - current value. Distance is found with the motor's integrated encoders and angle is found with the gyro/inertial sensor.

To reduce error precisely, PID adds up three variables to obtain a final speed output.
Each variable is multiplied by a corresponding tuning constant that affects how much the term affects the final speed (Kp, Ki, and Kd). The motor's speed is updated, and the cycle repeats.

### P: Proportional
- If you’re not where you want to be, get there. Go faster if you're far and slower if you're close.
- Where most of the speed comes from.
- Speed is proportional to error.

### I: Integral
- If you haven’t been where you want to be for a long time, get there faster.
- Makes sure speed is fast enough to overcome friction & weight of robot (called steady-state error).
- Accumulates error over time.

### D: Derivative
- If you’re getting close to where you want to be, slow down.
- Smooths out any unwanted oscillations and overshoot.
- Based on rate of change of the error.

### Tuning

At the top of fwdPIDCycle and turnPIDCycle are the tuning constants. I usually start with values around these:

Forwards:
```
Kp: start at 5
Ki: start at 0, it usually ends up at something small like 0.01
Kd: start at 1
```
Turning:
```
Kp: start at 0.6
Ki: start at 0, it usually ends up at something small like 0.001
Kd: start at 0.5
```

I usually start with Kp and make sure it's enough to get to the target. Then, increase Kd if the robot overshoots and oscillates around the target. Afterwards, if it is too slow when near the target, increase Ki to make it reach the target a little faster.


### PID Pseudocode
```
last_error = 0;
integral = 0;

while (running)
{
  error = target – current; // Obtain current error based on target value minus actual value
  
  integral = integral + error; // Accumulate the error
  
  derivative = error – last_error; // Find the rate of change of the error
  
  speed = Kp*error + Ki*integral + Kd*derivative; // Multiply each variable by its tuning constant and sum them up
  last_error = error;
}
```



## Odometry

Traditionally, VEX robots are controlled with commands to move relative to its current position. *Move forward, backward, turn 90 degrees, etc*. However, any obstacle in the robot's path could alter its direction and the code wouldn't be able to tell the difference. Odometry lets the code track the robot's x and y coordinates, which allows the path to be corrected. It is also more accurate to move in arc-shaped paths when odometry data is collected.

### Conventions
Certain math conventions must be followed when developing or using odometry. I used conventions that are commonly accepted in math:
- +x is right, -x is left, +y is up, -y is down
- The initial position of the robot is (0, 0)
- 0 degrees is on the positive x-axis
- Positive angles are counter-clockwise from 0 degrees

![image](https://user-images.githubusercontent.com/54510965/195655959-7456d2ae-fc32-4738-9304-768573737dc3.png)


### Development

We used the base of the Change Up bot as a mini bot to test odometry. It has unpowered tracking wheels hooked up to encoders.

Then I started the code by making a display for the robot's position and angle. The position, rotation, and encoder values are printed on the side.

![image](https://user-images.githubusercontent.com/54510965/195656350-cfdcb9fe-244f-4b95-9398-cb46614eae9f.png)

I then worked on turning towards a point. Inverse tangent gives the angle you have to turn to in order to face a point when given a relative x and y position. Then the robot finds which way to rotate to get to that angle. The angleWrap() function makes it so the robot doesn't turn over 180 degrees because it can turn less in the other direction to end up at the same angle.

In this pic the robot chooses to turn on the green shorter arc and not the red longer arc.

<img src="https://user-images.githubusercontent.com/54510965/195656001-742022d7-2d7f-4dbe-befc-8a96bdf91f2d.png" width="350">

Afterwards, the position tracking itself had to be worked on. The code gets the change in tracking wheel position and the current angle at every update. Then the distance and angle (called polar coordinates) are converted to x-y coordinates and added to the global position.

Here is a diagram of the odometry system.

<img src="https://user-images.githubusercontent.com/54510965/195656704-e79e0b20-702d-48ab-84bf-3f2db0dbb829.png" width="250">

The robot is moving in the direction of A. Theta is the angle of the inertial/gyro sensor. Ax and Ay are the x and y components of the movement. They are found with trig using the length of A and theta. Each update, Ax and Ay are added to globalX and globalY.

### Accuracy
A good odometry system should be able to move to the same place when given the same coordinates. However, in testing, the robot doesn't return exactly to its original position. It ends up a few inches away. However, as long as the robot returns to the same wrong position each time, it may be acceptable. For now, I have to accept that there's some arbitraity to guessing the positions.

The tracking wheels are important for accuracy and precision. The tracking wheels should be tensioned to the ground with rubber bands so if the robot jumps or skips the tracking wheels are still touching the ground. The mini bot doesn't have this but we'll do it for the real bot. Since the robot can't move sideways, it only needs two forwards tracking wheels without an additional sideways one. There is some added accuracy for having the sideways one but it might not be worth it for the space.

A document by [The Pilons](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf) helped with more accurate tracking. The document can get pretty complicated but most of it is dedicated to making slight adjustments. It addressed these problems:
 - Getting the robot's angle with encoders. We already use the inertial sensor for the angle but if a team doesn't have one, then they could use encoders instead.
 - The tracking wheels are offcentered, so they don't track the exact center of the robot. The code can shift the position over by some amount based on the arc of the movement. Accounting for this helped a bit but it's still not perfect.


### How to use
Moving to a point first involves a command to set the target position. Afterwards, you may move to or turn to that target point. Sometimes you need to lower the forward speed or raise the turning speed if the robot orbits around the target without stopping.
 
```
setTarget(10, 10); // Set the target position to x = 10 inches, y = 10 inches
moveToTarget(30, 15); // Move to the target at 30% forward speed and 15% turning speed. 

setTarget(0, 0); // Set the target position back to original position (0, 0) 
moveToTargetRev(25, 5); // Move forward to (0, 0) at 25% backward speed and 5% turning speed. 
```

You can pass by a target position without stopping at it with the following code:

```
setTarget(10, 10); // Set the target position to x = 10 inches, y = 10 inches
passToTarget(30, 15); // Move to the target at 30% forward speed and 15% turning speed. 

setTarget(0, 0); // Set the target position back to original position (0, 0) 
passToTargetRev(25, 5); // Back up to (0, 0) at 25% backward speed and 5% turning speed. 
```

## Multitasking In Autonomous
Multitasking involves invoking a function while another function is running. To call a function after moving some distance while moving, create the following function 

```
int startIntake10() 
{
  while (totalDistance < 10) // Distance (inches) can be anything 
  {
    // Wait inside the while loop until 10 inches is passed
    task::sleep(10);
  }
  
  setIntake(100); // This can be anything 
}
 ```

In autonomous, reset the distance tracker and start the task.

```
...
resetTotalDistance()
task a(startIntake10); // Setup the task to start the intake after 10 inches 

forwardPID(20, 25); // Move forward 20 inches (intake will start after 10 inches)
... 
```
 


## Pneumatics

### Driver control
```
// Create function for toggling pneumatic on/off 

void togglePneumatic() 
{ 
  pneumatic.set(!pneumatic.value());
} 
```

Scroll down to int main() at the bottom of the file. Add a callback for a  button to toggle the pneumatic.
This means the button assigned to call the function anytime it is pressed in the future.

```
int main() 
{
  ...
  // Create a callback for toggling pneumatic when A button pressed on primary controller
  ControllerPrim.ButtonA.pressed(togglePneumatic); 
  ...
}
```




 
