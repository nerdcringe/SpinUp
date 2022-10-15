using namespace vex;

extern brain Brain;
extern int hello1;

// Motors

extern motor L1BASE;
extern motor L2BASE; // back top
extern motor L3BASE; // back mid
extern motor L4BASE; // back bottom

extern motor R1BASE; // front
extern motor R2BASE; 
extern motor R3BASE;
extern motor R4BASE;


// Pneumatics

extern digital_out PWT;



// Sensors

extern inertial INERTIAL;

//extern triport Triport; // Get reference for three-wire ports on brain

extern encoder encoderL; // left tracking wheel
extern encoder encoderR; // right tracking wheel

extern controller controllerPrim;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
void newFunction();