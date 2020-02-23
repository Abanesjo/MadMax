#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftFront = motor(PORT12, ratio18_1, false);
motor RightFront = motor(PORT19, ratio18_1, true);
motor LeftBack = motor(PORT13, ratio18_1, false);
motor RightBack = motor(PORT20, ratio18_1, true);
motor armLift = motor(PORT1, ratio18_1, false);
motor armLeft = motor(PORT4, ratio18_1, true);
motor armRight = motor(PORT6, ratio18_1, false);
motor ramp = motor(PORT2, ratio18_1, false);
controller Controller1 = controller(primary);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}