using namespace vex;

extern brain Brain;

// VEXcode devices
extern encoder distanR;
extern inertial inert;
extern encoder distanL;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );