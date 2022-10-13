# VEX Team 1575Y Code - Spin Up 2022-23

This year for Spin-Up we wanted to prioritize accuracy because we're launching discs. Precise positioning is important for hitting the goals. We worked on a position tracking system, called odometry, for more accurate movements. Odometry allows the robot to move based on a system of x-y coordinates. Odometry also makes curved paths more accurate.


# The Code

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

 
# Guide

## PID Controllers

A PID controller is an algorithm we use to move motors precisely. The default motor functions VexCode provides aren't very precise. PID allows you to customize the way a motor moves to make it more accurate for its use case. We use PID to move straight and turn. It can also be used for moving lifts, maintaining a constant flywheel speed, and more.

In PID, the distance or angle from a target is called *error*. The goal of PID is to reduce error as precisely as possible. Error is calculated with the formula: desired value - current value. Distance is found with the motor's integrated encoders and angle is found with the gyro/inertial sensor.

To reduce error precisely, PID adds up three variables to obtain a final speed value.
Each term is multiplied by a corresponding tuning constant that affects how much the term affects the final speed (Kp, Ki, and Kd). The motor's speed is updated, and the cycle rapidly repeats.


### P: Proportional
- Speed is proportional to error
- Fast when error is high and slow when error is low
- Found by multiplying the error by Kp
- Tuning:
  - Increase if not fully reaching target.
  - Decrease if overshooting slightly.

### I: Integral
- Speed is the accumulated error
- Accumulates if far away from the target for a while
- When slowing down at the end from P, I makes sure speed is fast enough to overcome friction & weight of robot
- Tuning:
  - Increase Ki if the robot stops before the target.
  - Decrease Ki if the target is overshot and it takes a while to correct itself

### D: Derivative
- Speed is rate of change of the error
- Smooths out rapid changes in error when moving quickly due to P and I
- Dampens any unwanted oscillations and overshoot
- Tuning:
  - Increase Kd if the robot overshoots or oscillates around the target.
  - Decrease Kd if the robot stops before the target.

Generally, P and I increase speed and D decreases speed. P is the majority of the speed, especially at the start of the movement. Then, towards the middle, it should slow down until I kicks in and speeds it up slightly. At the end, D reduces overshoot and oscillations around the target value. It takes practice to intuitively understand the nuances of each variable.


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
I started by making this little GUI to graphically display the robot's position and angle. The position, rotation, and encoder values are printed.

![image](https://user-images.githubusercontent.com/54510965/195656350-cfdcb9fe-244f-4b95-9398-cb46614eae9f.png)

I then worked on turning towards a point. Inverse tangent gives the angle you have to turn to in order to face a point when given a relative x and y position. Then the robot finds which way to rotate to get to that angle. The angleWrap() function makes it so the robot doesn't turn over 180 degrees because it can turn less in the other direction to end up at the same angle.

In this pic the robot chooses to turn on the green shorter arc and not the red longer arc.

<img src="https://user-images.githubusercontent.com/54510965/195656001-742022d7-2d7f-4dbe-befc-8a96bdf91f2d.png" width="450">

Afterwards, the position tracking itself had to be worked on. The code gets the change in tracking wheel position and the current angle at every update. Then, those polar coordinates (defined by distance and angle instead of x and y) are converted to x, y coordinates and added to the global position.

Here is a diagram of the odometry system.

<img src="https://user-images.githubusercontent.com/54510965/195656704-e79e0b20-702d-48ab-84bf-3f2db0dbb829.png" width="250">

The robot is moving in the direction of A. Theta is the angle of the inertial/gyro sensor. Ax and Ay are the x and y components of the movement. They are found with trig using the length of A and theta. Each update, Ax and Ay are added to globalX and globalY.

### Accuracy
A good odometry system should be able to move to the same place when given the same coordinates. However, when going back to (0, 0), the robot doesn't return exactly to its original position. I ends up a few inches away. The longer the robot moves, the more innacurrate the position becomes. However, as long as the robot returns to the same wrong position each time, it is acceptable to some degree. We still needed solutions to minimize innacuracies.

A document by [The Pilons](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf) helped with more accurate tracking. It showed how to fix two the problems:
 - Getting the robot's angle with encoders. We already use the inertial sensor for the angle but if a team doesn't have one, then they could use encoders instead.
 - The tracking wheels are offcentered, so they don't track the exact center of the robot. It uses a separate calculation to try to adjust for this.
 
I tried accounting for the tracking wheels like The Pilons said and it helped the accuracy a bit. 

Since we can't move sideways, the robot technically only needs two parallel tracking wheels instead of an additional sideways one. There is some slight added accuracy for having the sideways one but it might not be worth it for the space.



### How to use
Moving to a point first involves a command to set the target position. Afterwards, you may move to or turn to that target point.
 
```
setTarget(10, 10); // Set the target position to x=10, y=10 
moveToTarget(30, 15); // Move to the target at 30% forward speed and 15% turning speed. 

setTarget(0, 0); // Set the target position back to original position (0, 0) 
moveToTargetRev(25, 5); // Move to the target in reverse at 25% backward speed and 5% turning speed. 
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




 
