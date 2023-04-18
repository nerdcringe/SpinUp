using namespace vex;

extern brain Brain;

// define your global instances of motors and other devices here
extern controller controllerPrim;

// MOTORS ////////////////////////////////////////
extern motor L1BASE;
extern motor L2BASE;
extern motor L3BASE;
extern motor LUPPER;

extern motor R1BASE;
extern motor R2BASE;
extern motor R3BASE;
extern motor RUPPER;

// PNEUMATICS ////////////////////////////////////
// Get reference for 3-wire ports on brain
// Its called PORT22 for some reason.
///triport Triport(PORT22);
extern digital_out BASEPTO;
extern digital_out LOCK;
extern digital_out ENDGAME1;
extern digital_out ENDGAME2;
extern digital_out BANDPTO;
extern digital_out MUNCH;

// SENSORS ////////////////////////////////////////////////
extern limit LIMIT;
extern inertial INERTIAL; // inertial sensor for tracking rotation




/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
