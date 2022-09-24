# 1575Y Vex Team Code


## Important Variables 

#### double *globalX*, *globalY*
The position of the robot relative to its starting position. 
 

#### double *targetX*, *targetY*
The current desired position of the robot that odometry movement functions will reference when invoked. 
 

#### double *totalDistance*
The total distance accumulated since the start of the current movement. 

 
 
## Movement Functions 

#### forwardPID 

- Parameters: double distance (inches), double maxSpeed (percent),  

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

- Parameters: double turnSpeed (percent) 

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

 

 

## PID

PID is a way to move some distance or turn to some angle.
The distance or angle from a target is called error. To reduce error precisely, PID adds up three variables to obtain a final speed value.

#### P: Proportional
- Speed is proportional to error
- Fast when error is high and slow when eror is low
- Majority of the speed comes from P term

#### I: Integral
- Speed is the accumulated error
- Accumulates if far away from the target for a while
- When slowing down at the end from P, I makes sure speed is fast enough to overcome friction & weight of robot

#### D: Derivative
- Speed is rate of change of the error
- If position is changing too fast, smooth out those changes
- Dampens any unwanted oscillations while trying to settle towards the target


Each term has a corresponding tuning constant that affects how much the term affects the final speed (Kp, Ki, and Kd).
If you want the speed to be purely proportional to distance, set Kp to any number but set Ki and Kd to 0.

##### PID Pseudocode
```
last_error = 0;
integral = 0;

while (running)
{
  error = target – current; // Obtain current error based on target value minus actual value
  
  integral = integral + error; // Accumulate the error
  
  derivative = error – last_error; // Find the rate of change of the error
  
  speed = Kp*error + Ki*integral + Kd*derivative;
  last_error = error;
}
```

The PID values for moving a set number of inches is around
```
float Kp = 5;
float Ki = 0.01;
float Kd = 1;
```

The PID values for turning to some degree is around
```
float Kp = 0.6;
float Ki = 0.001;
float Kd = 0.5;
```



## Using odometry in autonomous
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

linearPID(20, 25); // Move forward 20 inches (intake will start after 10 inches)
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




 
