using namespace vex;

extern brain Brain;

extern motor L1BASE;
extern motor L2BASE;
extern motor L3BASE;
extern motor R1BASE;
extern motor R2BASE;
extern motor R3BASE;
 
extern motor_group LBASE;
extern motor_group RBASE;

extern inertial INERTIAL;
 
extern controller controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
