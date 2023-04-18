#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

controller controller1(primary);

// MOTORS ////////////////////////////////////////
motor L1BASE(PORT17, true);
motor L2BASE(PORT20, true);
motor L3BASE(PORT16, true);
motor R1BASE(PORT11);
motor R2BASE(PORT12);
motor R3BASE(PORT2);

motor_group LBASE(L1BASE, L2BASE, L3BASE);
motor_group RBASE(R1BASE, R2BASE, R3BASE);

// SENSORS ///////////////////////////////////////
inertial INERTIAL(PORT6);

// PNEUMATICS ////////////////////////////////
//pneumatics PISTON(Brain.ThreeWirePort.A);


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}