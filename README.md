# 1575Y Vex Team Code

## Overview

This year for Spin-Up we wanted to prioritize accuracy because we're launching discs. Having the right position and orientation is important for hitting the goals. We worked on a position tracking system, called odometry, for more accurate movements. Odometry allows the robot to move based on a system of (x, y) coordinates.



## Important Variables 

#### double *globalX*, *globalY*
The position of the robot relative to its starting position. 
 

#### double *targetX*, *targetY*
The current desired position of the robot that odometry movement functions will reference.
 

#### double *totalDistance*
The total distance accumulated since the start of the current movement. 


 
## Movement Functions 

#### forwardPID 

- Parameters: double *distance* (inches), double *maxSpeed* (percent) 

- Description: Move forward/backwards for a certain distance using PID algorithm. Limit the output speed of the PID to a certain maximum speed. 

 

#### turnPID 

- Parameters: double *angle* (degrees), double *maxSpeed* (percent) 

- Description: Turn to a certain angle relative to the starting angle of the robot based on a PID algorithm. Limit the output speed of the PID to a certain maximum speed. 

 

#### forwardRamp 

- Parameters: double *distance* (inches), double *maxSpeed* (percent), int *timeoutMillis* (milliseconds) 

- Description: Move forward/backwards for a certain distance at a speed that ramps up, maxes out and slows back down at the end. Use this for smaller distances that PID cannot handle well. Exit after a certain time (timeoutMs). 

 
 

## Odometry Functions

#### setTarget 

- Parameters: double *x* (inches), double *y* (inches) 

- Description: Set the desired absolute target position on the field. 

 

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

 
## Sensor Functions

#### double getDegrees 
- Description: Return the current angle of the inertial sensor in degrees.


#### double getRadians 
- Description: Return the current angle of the inertial sensor in radians.


#### double angleWrap 
- Parameters: double *angle* (degrees)
- Description: Return the angle but within a -180 to 180 degree range of the robot. This makes it so the robot doesn't turn over 180 degrees since it can turn faster by turning the othe direction.
- Used to calculate the nearest angle to a point for odometryy movement
 

## PID

PID is an algorithm to get motors to move some distance or turn to some angle accurately. The default movement functions VexCode provides aren't completely accurate all the time. PID allows you to customize the way a motor moves to make it more accurate for its use case. PID can be used for base movement, lifts, maintaining a constant flywheel speed, and more. We have used PID in the past to move straight and turn, but we are adding the ability to move and turn simultaneously in a curved path using odometry.

In PID, the distance or angle from a target is called *error*. Error is calculated with the formula: desired value - current value.
Distance error is found with the integrated motor encoders and angle error is found with the gyo/inertial sensor.

To reduce error precisely, PID adds up three variables to obtain a final speed value.
Each term is multiplied by a corresponding tuning constant that affects how much the term affects the final speed (Kp, Ki, and Kd). The motor's speed is updated, and after a few milliseconds, the cycle repeats.


### P: Proportional
- Speed is proportional to error
- Fast when error is high and slow when error is low
- Found by multiplying the error by Kp

##### Tuning
 - Increase if not fully reaching target.
 - Decrease if overshooting slightly.


### I: Integral
- Speed is the accumulated error
- Accumulates if far away from the target for a while
- When slowing down at the end from P, I makes sure speed is fast enough to overcome friction & weight of robot

##### Tuning
- Increase Ki if the robot stops before the target.
- Decrease Ki if the target is overshot and it takes a while to correct itself

### D: Derivative
- Speed is rate of change of the error
- Smooths out rapid changes in error when moving quickly due to P and I
- Dampens any unwanted oscillations and overshoot

##### Tuning
- Increase Kd if the robot overshoots or oscillates around the target.
- Decrease Kd if the robot stops before the target.

Generally, P and I increase speed and D decreases speed. P is the majority of the speed, especially at the start of the movement. Then, towards the middle, it should slow down until I kicks in and speeds it up slightly. At the end, D reduces overshoot and oscillations around the target value. It takes practice to intuitively understand the nuances of each variable.


##### PID Pseudocode
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

Approximate PID tuning values for distance (inches)
```
float Kp = 5;
float Ki = 0.01;
float Kd = 1;
```

Approximate PID tuning values for turning to a degree
```
float Kp = 0.6;
float Ki = 0.001;
float Kd = 0.5;
```



## Using odometry in autonomous

Traditionally, VEX robots are controlled with commands to move relative to its current position. *Move forward, backward, turn 90 degrees, etc*. However, any obstacle in the robot's path could alter its direction and the code wouldn't be able to tell the difference. Odometry lets the code track the robot's x and y coordinates, which allows the path to be corrected. It is also more accurate to move in arc-shaped paths when odometry data is collected.

To use a comprehensive and reliable odometry system, certain math conventions must be followed:
- +x is right, -x is left, +y is up, -y is down
- The initial position of the robot is (0, 0)
- 0 degrees is on the positive x-axis
- Positive angles are counter-clockwise from 0 degrees



Moving to a point first involves a command to set the target position. Afterwards, you may move to or turn to that target point.
 
```
setTarget(10, 10); // Set the target position to x=10, y=10 
moveToTarget(30, 15); // Move to the target at 30% forward speed and 15% turning speed. 

setTarget(0, 0); // Set the target position back to original position (0, 0) 
moveToTargetRev(25, 5); // Move to the target in reverse at 25% backward speed and 5% turning speed. 
```

## Multitasking in autonomous
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
 


## Adding pneumatics to driver control

```
// Create function for toggling pneumatic on/off 

void togglePneumatic() 
{ 
  if (pneumatic.value() == 1) 
  { 
    pneumatic.set(0);
  }
  else
  { 
    pneumatic.set(1) 
  } 
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




 
