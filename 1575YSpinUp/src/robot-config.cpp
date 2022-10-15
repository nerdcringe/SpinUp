#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

int hello1 = 0;

 // motors //

motor L1BASE(PORT15, true); // front
motor L2BASE(PORT17, true); // back top
motor L3BASE(PORT14); // back mid
motor L4BASE(PORT11, true); // back bottom

motor R1BASE(PORT18); // front
motor R2BASE(PORT13); 
motor R3BASE(PORT19, true);
motor R4BASE(PORT9);


// Sensors & more //

inertial INERTIAL(PORT14);

triport Triport(PORT22); // Get reference for three-wire ports on brain

digital_out PWT(Triport.A);


triport TriportExt(PORT9); // Get reference for three wire extender
encoder encoderL(TriportExt.A); // left tracking wheel
encoder encoderR(TriportExt.C); // right tracking wheel

controller controllerPrim(controllerType::primary);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}

void newFunction()
{

}