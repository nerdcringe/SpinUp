#include "vex.h"
//#include "testVision.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;


motor RBASEMotorA = motor(PORT18, ratio18_1, true);
motor RBASEMotorB = motor(PORT19, ratio18_1, true);
motor_group RBASE = motor_group(RBASEMotorA, RBASEMotorB);
motor LBASEMotorA = motor(PORT13, ratio18_1, false);
motor LBASEMotorB = motor(PORT17, ratio18_1, false);
motor_group LBASE = motor_group(LBASEMotorA, LBASEMotorB);
controller Controller1 = controller(primary);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}