#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;



// define your global instances of motors and other devices here
controller controllerPrim(primary);

// MOTORS ////////////////////////////////////////
motor L1BASE(PORT17, true);
motor L2BASE(PORT20, true);
motor L3BASE(PORT16, true);
motor LUPPER(PORT15, true);

motor R1BASE(PORT11);
motor R2BASE(PORT12);
motor R3BASE(PORT2);
motor RUPPER(PORT14);

// PNEUMATICS ////////////////////////////////////
// Get reference for 3-wire ports on brain
// Its called PORT22 for some reason.
triport Triport(PORT22);
digital_out BASEPTO(Triport.A);
digital_out LOCK(Triport.B);
digital_out ENDGAME1(Triport.D);
digital_out ENDGAME2(Triport.E);
digital_out BANDPTO(Triport.F);
digital_out MUNCH(Triport.G);

// SENSORS ////////////////////////////////////////////////
limit LIMIT(Triport.C);
inertial INERTIAL(PORT6); // inertial sensor for tracking rotation




/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}