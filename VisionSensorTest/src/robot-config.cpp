#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
/*vex-vision-config:begin*/
signature Vision__REDGOAL = signature (1, 5831, 8223, 7026, -607, 529, -38, 2.4, 0);
signature Vision__BLUEGOAL = signature (2, -2739, -2093, -2416, 6499, 9143, 7820, 3, 0);
signature Vision__SIG_3 = signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_7 = signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vision Vision = vision (PORT14, 50, Vision__REDGOAL, Vision__BLUEGOAL, Vision__SIG_3, Vision__SIG_4, Vision__SIG_5, Vision__SIG_6, Vision__SIG_7);
/*vex-vision-config:end*/
motor RBASEMotorA = motor(PORT18, ratio18_1, true);
motor RBASEMotorB = motor(PORT19, ratio18_1, true);
motor_group RBASE = motor_group(RBASEMotorA, RBASEMotorB);
motor LBASEMotorA = motor(PORT13, ratio18_1, false);
motor LBASEMotorB = motor(PORT17, ratio18_1, false);
motor_group LBASE = motor_group(LBASEMotorA, LBASEMotorB);
controller Controller1 = controller(primary);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}