using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LeftFront;
extern motor RightFront;
extern motor LeftBack;
extern motor RightBack;
extern motor armLift;
extern motor armLeft;
extern motor armRight;
extern motor ramp;
extern controller Controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );