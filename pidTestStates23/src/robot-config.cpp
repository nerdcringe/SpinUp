/* 
* THIS IS A FILE FOR CONSTRUCTING/CONFIGURING ALL THE PHYSCIAL HARDWARE YOU DECLARED IN robot-config.h:
 
 Think of this file as completing the promise you made in robot-config.h and configuring the details of the hardware defined there.
 Essentially we are bringing the objects we create to life so we have to make sure we use the same names or else the compiler will be confused
*/


#include "vex.h"

// This means that you will be using functions and objects from the vex library. In the vex v5 api it often shows things like "vex::motor". This is explicitly telling the compiler that 
//    we are using the motor object from the vex library. If we say we are using namespace vex that means the compile assumes we are using vex objects if we dont tell it otherwise.
//    WE DO NOT NEED TO CHANGE THIS LINE
using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

int hello1 = 0;

// real bot motors

motor L1BASE(PORT11, true);
motor L2BASE(PORT17);
motor L3BASE(PORT14, true);

motor R1BASE(PORT13);
motor R2BASE(PORT3, true);
motor R3BASE(PORT19);

motor INTAKE(PORT15, true);
motor XBOW(PORT18, true);


// mini bot

// motor L1BASE(PORT1); // front
// motor L3BASE(PORT17); // back top

// motor R1BASE(PORT18, true);
// motor R3BASE(PORT13, true); 


// motor L2BASE(PORT14); // back mid
// motor L4BASE(PORT11, true); // back bottom
// motor R2BASE(PORT19, true);
// motor R4BASE(PORT10);


/* THREE-WIRE PORTS
    Old sensors still use three-wire ports like on the old system.
*/
triport Triport(PORT22); // Get reference for 3-wire ports on brain (arbitrarily called PORT22 but PORT22 doesn't really exist)
triport TriportExt(PORT1); // Get reference for 3 wire extender ports

// PNEUMATICS
digital_out ENDGAME(Triport.A);
digital_out LOCK(Triport.B); 
digital_out PTO(Triport.E); // power take off for rubber bands
digital_out MUNCHER(Triport.F); // 3-stack mech

// SENSORS
inertial INERTIAL(PORT21); // inertial sensor for tracking rotation
bumper CATALIMIT(Triport.D); // bump switch for pullback

// CONTROLLER
controller controllerPrim(controllerType::primary);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize

}

void newFunction() {
}




/* 
VEXcode device constructors. This is where we construct/configure all of the hardware we said we would use in robot-config.h 
The general way to constuct an object is as follows:
  - objectType objectName = objectType(arg1, arg2, arg3)
Different objects have different constructors. For a complete list of constructors check here: https://api.vexcode.cloud/v5/
  - click that link, click on the object you want to create on the left, and use the constructor provided there
Heres an example of a motorGroup and motor constructor:
  - motor motor1 = motor(1, gearSetting::ratio18_1, true)
  - motor motor2 = motor(1, gearSetting::ratio18_1, false)
  - motor_group MG1 = motor_group(motor1, motor2);
Breaking down each word for motor1 instanitation is:
  "motor"                         <-- The type, tells the compile you are instantiating a motor
  "motor1"                        <-- The name of the motor. You will refer to this motor as this name for the rest of the code
  "motor("                        <-- The start of the constructor for the motor
  "1"                             <-- The port the motor is plugged into
  "geatSetting::ratio18_1"        <-- The gear setting for the motor (ratio18_1 is green, ratio36_1 is red, ratio6_1 is blue) 
  "true"                          <-- Whether the motor is reversed. true means this motor is reverse, false means the motor is not reversed.
Breaking down each word for motor_group instantiation is:
  "motor_group"                   <-- The type, tells the compiler you are instantiating a motor group
  "MG1"                           <-- The name of the motor group. For the rest of the code you will refer to it with this name
  "motor_group("                  <-- The start of the constructor for the motor_group
  "motor1, motor2"                <-- The motors that make up the motor group (you can have any number of motors in the group)
  BELOW ARE A FEW EXAMPLES OF motor, motor_group, and sensor INSTANTIATIONS
*/
