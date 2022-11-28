/* 
* THIS IS A FILE FOR DECLARING ALL THE PHYSCIAL HARDWARE YOU WILL BE USING SUCH AS:
  - Motor
  - MotorGroup
  - Sensors
  - Controllers
  - Brain
 
 Think of this file as making a promise that you will use only the hardware defined here.
*/


// This means that you will be using functions and objects from the vex library. In the vex v5 api it often shows things like "vex::motor". This is explicitly telling the compiler that 
//    we are using the motor object from the vex library. If we say we are using namespace vex that means the compile assumes we are using vex objects if we dont tell it otherwise.
//    WE DO NOT NEED TO CHANGE THIS LINE
using namespace vex;

extern brain Brain;
extern int hello1;


// MOTORS

extern motor L1BASE; // front
extern motor L2BASE; // back top
extern motor L3BASE; // back mid
//extern motor L4BASE; // back bottom

extern motor R1BASE;
extern motor R2BASE;
extern motor R3BASE;
//extern motor R4BASE;

extern motor INTAKE;
extern motor XBOW;



// PNEUMATICS ////////////////////
extern digital_out LOCK;
extern digital_out PTO;
extern digital_out ENDGAME;
////////////////////////////////////////////

// SENSORS /////////////////////////////////////
extern inertial INERTIAL; // gives angle of robot
//extern rotation CATAPOT; // gives angle of catapult for auto-loading
extern limit CATALIMIT;
extern encoder encoderL; // left tracking wheel
extern encoder encoderR; // right tracking wheel
////////////////////////////////////////////

//////////  CONTROLLER  DECLARATION /////////
/*
  Controller declaration is similar to previous declarations
*/
extern controller controllerPrim;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
void newFunction();



//////////  MOTOR DECLARATION /////////
/*
  In VexV5 Pro you can decalre Motor and MotorGroup
  - Motor       <-- Used to declare single motor
  - MotorGroup  <-- Used to declare multiple motors that will be working together in 1 sub system
  NOTE: You have to declare the motors you plan on using in the motor group. So if you have a leftDrive motor group you also need to separately declare the motors that would be in that group
  
  The general way to declare a Motor is as follows:
    - extern Motor motorName
  The general way to declare a MotorGroup is as follows:
    - extern MotorGroup motorGroupName
  Breaking down each word:
    - "extern"                        <--  tells the compiler that this is a thing that will be referenced in another file
    - "Motor" or "MotorGroup"         <-- is the thing you're declaring
    - "MotorName" or "motorGroupName" <-- is the name you will be using to reference that object for the rest of the code
  BELOW ARE A FEW EXAMPLES OF Motor and MotorGroup DECLARATIONS

  We're not using motorGroups now because we're switching between 8 motor and 6 motor base + catapult + rollers.
    Pneumatics will move a gear to switch motors between the base and catapult/roller.
    This is commonly called power take-off (PTO).
    So we need to activate the motors individually based on if they're part of the base or not at that current moment.
*/




/////////   SENSOR DECLARATIONS   ///////////
/*
  There are a few different types of sensors we can use. Some sensors can be declared in a few different ways. For example, the Potentiometer is an analog_in sensor but it is also a potentiometer.
    All sensors can be described as either analog or digital so in theory you could have all sensors be analog_in or digital_in but using the specific sensor class lets you use functions specific 
    to that sensor class.
    Ex:
      - If you declare a potentiometer as an analog_in you can only use the 1 function that comes with that object (changed - function that checks when the analog_in value changed) but if you declare
          the potentiometer as a potentiometer then you can use the 2 functions that come with it (changed and angle - function that gives you current angle of the potentiometer)
  You can declare the following sensors:
  - analog_in     <-- Anything that is an analog sensor like a line follower or potentiometer (I dont reccommend you use this unless you are making your own sensors or using 3rd party sensors for VEXU)
  - bumper        <-- Used to delare a bumper sensor
  - digital_in    <-- Anything that is a digital sensor like a limit switch (I dont reccommend you use this unless you are making your own sensors or using 3rd party sensors for VEXU)
  - digital_out   <-- Anything that takes a digital value (really just pneumatic solenoids but theres a pnematics object so you can just use that)
  - distance      <-- Used to declare the distance sensor
  - encoder       <-- Used to declare the encoder sensor
  - gps           <-- Used to declare the GPS sensor
  - gyro          <-- Used to delcare the gyro sensor
  - intertial     <-- Used to declare the inertial sensor
  - light         <-- Used to declare the light sensor
  - limit         <-- Used to declare the limit switch sensor
  - line          <-- Used to declare the line sensor
  - optical       <-- Used to declare the optical sensor
  - pneumatics    <-- Used to declare the pneuamtic solenoid sensor
  - pot           <-- Used to declare the potentiometer sensor
  - rotation      <-- Used to delcare the rotation sensor
  - sonar         <-- Used to declare the sonar sensor
  - vision        <-- Used to declare the vison sensor
  Heres the link to all devices you could declare, how to declare them, and their functions:
    https://api.vexcode.cloud/v5/abstract/class/
  The general way to declare a sensor is as follows:
    - extern bumper sensorName;
    Breaking down each word:
      - "extern" tells the compiler that this is a thing that will be referenced in another file
      - "bumper" is the type of sensor you are declaring. This can be anything from the list above
      - "sensorName" is the name you will be using to refernce that sensor for the rest of the code
   BELOW ARE A FEW EXAMPLES OF sensor DECLARATIONS
*/
