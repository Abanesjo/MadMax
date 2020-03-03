/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         12              
// RightFront           motor         19              
// LeftBack             motor         13              
// RightBack            motor         20              
// armLift              motor         1               
// armLeft              motor         4               
// armRight             motor         6               
// ramp                 motor         2               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <chrono>
#include <iostream>
#include <cmath>
#include <cstdio>

using namespace vex;
using namespace std::chrono;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
#pragma region global
double armAngle = 0;
bool autoIntake = false;
bool manual = false;
int countr = 0;
int rampState = 0;
int mmax = 3;
int motors[2][3] = {
  {0,0,0},
  {0,0,0}
};

void controlMode() {
  manual = !manual;
}
void intakeMode() {
  autoIntake = !autoIntake;
}
#pragma endregion

#pragma region tools
void velocitySet(double left = 30, double right = 30) {
  LeftFront.setVelocity(left,percentUnits::pct);
  LeftBack.setVelocity(left,percentUnits::pct);
  RightFront.setVelocity(right,percentUnits::pct);
  RightBack.setVelocity(right,percentUnits::pct);
}
double toDeg(double cm) {
  double wheelDiameter = 10.16;
  double circumference = wheelDiameter * 3.141592;
  double degreesToRotate = 360 * cm/circumference;
  return degreesToRotate;
}
void printAngle(motor test) {
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(100,150,"Angle: %0.2lf",test.rotation(rotationUnits::deg));
}
double tocm(double deg) {//not used yet
  double wheelDiameter = 10.16;
  double circumference = wheelDiameter * 3.141592;
  double cm = deg * circumference /360;
  return cm;
}

void move(directionType type) { //not used
  LeftFront.spin(type);
  RightFront.spin(type);
  LeftBack.spin(type);
  RightBack.spin(type);
}
void stopp() {
  LeftFront.stop();
  RightFront.stop();
  LeftBack.stop();
  RightBack.stop();
}
void forwardDistance(double cm, double vel) {
  velocitySet(vel, vel);
  double deg = toDeg(cm);
  LeftFront.spinFor(deg,rotationUnits::deg,false);
  LeftBack.spinFor(deg,rotationUnits::deg,false);
  RightFront.spinFor(deg,rotationUnits::deg,false);
  RightBack.spinFor(deg,rotationUnits::deg);
}
void forwardTime(double durationMS, double vel) {
  velocitySet(vel, vel);
  LeftFront.spin(directionType::fwd);
  LeftBack.spin(directionType::fwd);
  RightFront.spin(directionType::fwd);
  RightBack.spin(directionType::fwd);
  task::sleep(durationMS);
  LeftFront.stop();
  RightFront.stop();
  LeftBack.stop();
  RightBack.stop();
}
void turn(bool direction_, double degrees, double vel) {
  velocitySet(vel,vel);
  if(!direction_) { //turns right
    LeftFront.spinFor(degrees, rotationUnits::deg, false);
    LeftBack.spinFor(degrees, rotationUnits::deg, false);
    RightFront.spinFor(-degrees, rotationUnits::deg, false);
    RightBack.spinFor(-degrees, rotationUnits::deg);
  } else {
    LeftFront.spinFor(-degrees, rotationUnits::deg, false);
    LeftBack.spinFor(-degrees, rotationUnits::deg, false);
    RightFront.spinFor(degrees, rotationUnits::deg, false);
    RightBack.spinFor(degrees, rotationUnits::deg);
  }
}
void accelerate(double u_percent,double v_percent, double x_cm) {
  double u = (u_percent/100) * 1200;
  double v = (v_percent/100) * 1200;
  double x = (x_cm /100)*(180/(M_PI*0.0508)); // decrease or change wheel radius
  double deltaT = 5;
  double vel = u;
  while(vel < v) {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10,14,"%0.2f",vel);

    LeftFront.spin(directionType::fwd,vel,velocityUnits::dps);
    RightFront.spin(directionType::fwd,vel,velocityUnits::dps);
    LeftBack.spin(directionType::fwd,vel,velocityUnits::dps);
    RightBack.spin(directionType::fwd,vel,velocityUnits::dps);

    wait(deltaT,timeUnits::msec);
    vel += (deltaT/1000)*((v*v)- (u*u)) / (2*x);
  }
  velocitySet(0,0);
}
void decelerate(double u_percent,double v_percent, double x_cm) {
  double u = (u_percent/100) * 1200;
  double v = (v_percent/100) * 1200;
  double x = (x_cm /100)*(180/(M_PI*0.0508)); // decrease or change wheel radius
  double deltaT = 5;
  double vel = u;
  while(vel > v) {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10,14,"%0.2f",vel);

    LeftFront.spin(directionType::fwd,vel,velocityUnits::dps);
    RightFront.spin(directionType::fwd,vel,velocityUnits::dps);
    LeftBack.spin(directionType::fwd,vel,velocityUnits::dps);
    RightBack.spin(directionType::fwd,vel,velocityUnits::dps);

    wait(deltaT,timeUnits::msec);
    vel += (deltaT/1000)*((v*v)- (u*u)) / (2*x);
  }
  velocitySet(0,0);
}

#pragma endregion

#pragma region actions
void tank() {
  LeftFront.setBrake(brakeType::hold);
  LeftBack.setBrake(brakeType::hold);
  RightFront.setBrake(brakeType::hold);
  RightBack.setBrake(brakeType::hold);
  LeftFront.spin(directionType::fwd, Controller1.Axis3.position()*0.6, percentUnits::pct);
  LeftBack.spin(directionType::fwd, Controller1.Axis3.position()*0.6, percentUnits::pct);
  RightFront.spin(directionType::fwd, Controller1.Axis2.position()*0.6, percentUnits::pct);
  RightBack.spin(directionType::fwd, Controller1.Axis2.position()*0.6, percentUnits::pct);
}

void userAcceleration(void){
  LeftFront.setBrake(brakeType::hold);
  LeftBack.setBrake(brakeType::hold);
  RightFront.setBrake(brakeType::hold);
  RightBack.setBrake(brakeType::hold);
  LeftFront.spin(directionType::fwd);
  RightFront.spin(directionType::fwd);
  int velocities[2] = {0,0};

  motors[0][0]=Controller1.Axis3.position(); // left
  motors[1][0]=Controller1.Axis2.position(); // right
  
  for(int m=0; m<2; m++){
    int mymot = motors[m][mmax-1];
    for(int i=mmax-2; i>=0; i--){
      mymot += motors[m][i]; //totaling 3 most recent axis positions
      motors[m][i+1] = motors[m][i]; //moving up axis position values
    }
    mymot /= (mmax+1); 
    velocities[m] = mymot;
    //averaging 3 most recent axis positions for left/right motors
  }
  velocitySet(velocities[0], velocities[1]); 
  LeftFront.spin(vex::directionType::fwd);
  RightFront.spin(vex::directionType::fwd);
  LeftBack.spin(vex::directionType::fwd);
  RightBack.spin(vex::directionType::fwd);

  vex::task::sleep(5);
}

void arm() {
 ramp.setBrake(brakeType::hold);
 armLift.setBrake(brakeType::hold);
 double rampVelocity = 30;
 double armLiftVelocity = 50;

 if(manual) {
   Controller1.Screen.clearScreen();
   Controller1.Screen.print("Manual Mode");
   if(Controller1.ButtonUp.pressing()) {
     ramp.spin(directionType::fwd,rampVelocity,percentUnits::pct);
   } else if(Controller1.ButtonDown.pressing()) {
     ramp.spin(directionType::rev,rampVelocity,percentUnits::pct);
   } else {
     ramp.stop();
   }

    if(Controller1.ButtonR1.pressing()) {
    armLift.spin(directionType::fwd,armLiftVelocity,percentUnits::pct);
  } else if(Controller1.ButtonR2.pressing()) {
    armLift.spin(directionType::rev,armLiftVelocity,percentUnits::pct);
  } else {
    armLift.stop();
   }

 } else { //automatic
    ramp.setVelocity(rampVelocity,percentUnits::pct);
    armLift.setVelocity(armLiftVelocity,percentUnits::pct);
    Controller1.Screen.clearScreen();
    Controller1.Screen.print("Automatic Mode");
    if(Controller1.ButtonUp.pressing()) {
      ramp.rotateTo(1300,rotationUnits::deg,false); //perpendicular position(stacking)
    } else if(Controller1.ButtonDown.pressing()) {
      ramp.rotateTo(10,rotationUnits::deg,false); //declined position(intake / rest)
    } else {
      //This controls the ramp when the arm is raised
      //Functions similarly to the one in manual control
      if(armLift.rotation(rotationUnits::deg) > 30) {
        countr = 1;
        ramp.spinTo(500,rotationUnits::deg,70,velocityUnits::pct,false);
      } else if(armLift.rotation(rotationUnits::deg) < 30 && countr == 1) {
        //"false" tag for this function now available, simplifying the issue a lot 
        ramp.rotateTo(10,rotationUnits::deg,70,velocityUnits::pct,false); 
        countr = 0;
      }
      //notice the lack of .stop() methods
    }
    //Controls the raising of the arm
    if(Controller1.ButtonX.pressing()) {
      armLift.rotateTo(1034,rotationUnits::deg,false); //middle-height tower
    } else if(Controller1.ButtonA.pressing()) {
      armLift.rotateTo(780,rotationUnits::deg,false); //lowest tower
    } else if(Controller1.ButtonB.pressing()) {
      armLift.rotateTo(10,rotationUnits::deg,false); //rest
    }
  }
  printAngle(ramp);

  if(Controller1.ButtonL1.pressing()) { //makes the rollers intake blocks
    armLeft.setVelocity(100,percentUnits::pct);
    armRight.setVelocity(100,percentUnits::pct);
    armLeft.spin(directionType::fwd);
    armRight.spin(directionType::fwd);
  } else if(Controller1.ButtonL2.pressing()) { //spins in reverse / outtake
    armLeft.setVelocity(100,percentUnits::pct);
    armRight.setVelocity(100,percentUnits::pct);
    armLeft.spin(directionType::rev);
    armRight.spin(directionType::rev);
  } else {
    if(autoIntake) { //intakeState changed by "intakeMode" seen at the bottom of the code
      armLeft.spin(directionType::fwd,100,velocityUnits::pct);
      armRight.spin(directionType::fwd,100,velocityUnits::pct);
    } else {
      armLeft.stop();
      armRight.stop();
    }
  }
}
#pragma endregion

#pragma region auton
void skillsixteen() {
  //------------Expansion----------------
  // armLeft.setVelocity(100,percentUnits::pct);
  // armRight.setVelocity(100,percentUnits::pct);
  // armLeft.spin(directionType::rev);
  // armRight.spin(directionType::rev);
  // task::sleep(500);
  // armLeft.spin(directionType::fwd);
  // armRight.spin(directionType::fwd);
  // forwardTime(300,-40);
  // task::sleep(150);
  //-------------Intake------------------
  armLeft.setVelocity(100,percentUnits::pct);
  armRight.setVelocity(100,percentUnits::pct);
  armLeft.spin(directionType::fwd);//
  armRight.spin(directionType::fwd);
  forwardDistance(75,15);
  task::sleep(500);
  forwardDistance(-10,20);
  task::sleep(500);
  turn(0,147,20);
  task::sleep(500);
  forwardDistance(-72,20);
  task::sleep(500);
  turn(1,150,20);
  task::sleep(500);


  forwardDistance(78,15);
  armLeft.setBrake(hold);
  armRight.setBrake(hold);
  armLeft.stop();
  armRight.stop();
  task::sleep(500);
  forwardDistance(-38,15);

  turn(1,380,20); //389
  task::sleep(500);

  //--------------Stacking----------------
  forwardTime(2500,50);
  task::sleep(500);

  forwardTime(500,-10);

  armLeft.setVelocity(25,percentUnits::pct);
  armRight.setVelocity(25,percentUnits::pct);
  armLeft.rotateFor(-75,rotationUnits::deg,false);
  armRight.rotateFor(-75,rotationUnits::deg);

  armLeft.stop();
  armRight.stop();

  task::sleep(500);

  armLeft.setVelocity(20,percentUnits::pct);
  armRight.setVelocity(20,percentUnits::pct);
  armLeft.spin(directionType::fwd);
  armRight.spin(directionType::fwd);
  ramp.setVelocity(30,percentUnits::pct);
  ramp.rotateTo(1070,rotationUnits::deg);
  task::sleep(500);
  forwardTime(800,10);
  task::sleep(500);
  armLeft.setVelocity(-15,percentUnits::pct);
  armRight.setVelocity(-15,percentUnits::pct);
  armLeft.rotateFor(-800,rotationUnits::deg,false);
  armRight.rotateFor(-800,rotationUnits::deg,false);
  forwardDistance(-30,30); // 25
  armLeft.stop();
  armRight.stop();

  //------------Scoring-------------
   task::sleep(500);
  turn(1,430,15); //maybe 405
  LeftFront.setVelocity(30,percentUnits::pct);
  LeftBack.setVelocity(30,percentUnits::pct);
  RightFront.setVelocity(30,percentUnits::pct);
  RightBack.setVelocity(30,percentUnits::pct);
  task::sleep(500);
  ramp.rotateTo(400,rotationUnits::deg,false);
  forwardTime(2500,-50);
  task::sleep(500);
  //----------------------------------------
  forwardDistance(100,25);
  task::sleep(500);
  armLeft.setVelocity(40,percentUnits::pct);
  armRight.setVelocity(40,percentUnits::pct);
  armLeft.rotateFor(300,rotationUnits::deg,false);
  armRight.rotateFor(300,rotationUnits::deg);

  task::sleep(500);
  forwardDistance(-10,20);
  task::sleep(500);

  armLift.setVelocity(50,percentUnits::pct);
  armLift.rotateTo(1100,rotationUnits::deg);
  task::sleep(200);
  forwardDistance(30,20);
  task::sleep(500);
  armLeft.setVelocity(-15,percentUnits::pct);
  armRight.setVelocity(-15,percentUnits::pct);
  armLeft.spin(directionType::fwd);
  armRight.spin(directionType::fwd);
  task::sleep(2000);
  forwardDistance(-80,30);
  armLift.stop();
  armLeft.stop();
  armRight.stop();
}
void bluEight() {
  //---------------Expansion-----------
  //expansion
  armLeft.setVelocity(100,percentUnits::pct);
  armRight.setVelocity(100,percentUnits::pct);
  armLeft.spin(directionType::rev);
  armRight.spin(directionType::rev);
  task::sleep(200);
  armLeft.spin(directionType::fwd);
  armRight.spin(directionType::fwd);
  forwardTime(100,-40);
  // task::sleep(100);

  //end of expansion
  //----------------intake--------------
  armLeft.setVelocity(100,percentUnits::pct);
  armRight.setVelocity(100,percentUnits::pct);
  armLeft.spin(directionType::fwd);//
  armRight.spin(directionType::fwd);

  forwardDistance(87,45);
  task::sleep(200);
  turn(1,120,40);
  task::sleep(200);
  forwardDistance(-90,50);
  task::sleep(150);
  turn(0,143,40);
  task::sleep(200);
  forwardDistance(85,30);
  task::sleep(100);
  forwardDistance(-10,20);
  armLeft.setBrake(hold);
  armRight.setBrake(hold);
  task::sleep(100);
  turn(1,375,30);
  task::sleep(150);


  armLeft.setVelocity(8,percentUnits::pct);
  armRight.setVelocity(8,percentUnits::pct);
  //----------------Stacking--------------
  forwardTime(1100,95);
  task::sleep(100);

  armLeft.setBrake(brakeType::coast);
  armRight.setBrake(brakeType::coast);
  armLeft.setVelocity(-8,percentUnits::pct);
  armRight.setVelocity(-8,percentUnits::pct);
  armLeft.spin(directionType::fwd);
  armRight.spin(directionType::fwd);
  ramp.setVelocity(50,percentUnits::pct);

  ramp.rotateTo(800,rotationUnits::deg);
  ramp.setVelocity(27,percentUnits::pct);
  ramp.rotateTo(1200,rotationUnits::deg);
  armLeft.stop();
  armRight.stop();
  task::sleep(100);
  armLeft.setVelocity(-20,percentUnits::pct);
  armLeft.rotateFor(800,rotationUnits::deg,false);
  armRight.rotateFor(800,rotationUnits::deg,false);
  ramp.setVelocity(100,percentUnits::pct);
  ramp.rotateTo(400,rotationUnits::deg,false);
  forwardDistance(-30,60); // 25
  armLeft.stop();
  armRight.stop();
}
void blueBoi() {
  LeftFront.setBrake(brakeType::brake);
  LeftBack.setBrake(brakeType::brake);
  RightFront.setBrake(brakeType::brake);
  RightBack.setBrake(brakeType::brake);
  //----------------Expansion--------------------
  // armLeft.setVelocity(100,percentUnits::pct);
  // armRight.setVelocity(100,percentUnits::pct);
  // armLeft.spin(directionType::rev);
  // armRight.spin(directionType::rev);
  // task::sleep(200);
  // armLeft.spin(directionType::fwd);
  // armRight.spin(directionType::fwd);
  // forwardTime(100,-40);
  // task::sleep(300);
  //----------------------------------------------
  armLeft.setVelocity(100,percentUnits::pct);
  armRight.setVelocity(100,percentUnits::pct);
  armLeft.spin(directionType::fwd);
  armRight.spin(directionType::fwd);
  //-----------------Intake-----------------------
  //forwardDistance(87,30);
  // accelerate(0,50,45);
  accelerate(30,50,45);
  // forwardDistance(45,45);
  decelerate(50,30,52);
  stopp();
  wait(150,timeUnits::msec);
  double left = -60;
  double right = -20;
  while(true) {
    velocitySet(left,right);
    LeftFront.spin(vex::directionType::fwd);
    LeftBack.spin(vex::directionType::fwd);
    RightFront.spin(vex::directionType::fwd);
    RightBack.spin(vex::directionType::fwd);
    wait(10.97,timeUnits::msec);
    left += 0.15;
    right -= 0.15;
    if(left > -21) {
      break;
    }
  }
  stopp();
  wait(500,timeUnits::msec);
  // forwardDistance(87,30);
  // accelerate(0,50,30);
  // forwardDistance(30,45);
  accelerate(30,50,30);
  decelerate(50,30,67);
  // forwardDistance(60,40);
  wait(100,timeUnits::msec);
  turn(1,380,30);
  wait(100,timeUnits::msec);

  // forwardTime(1500,70);
  decelerate(60,35,90);
  // armLeft.rotateFor(-180,rotationUnits::deg,false);
  // armRight.rotateFor(-180,rotationUnits::deg,false);
  forwardTime(900,30);
  wait(200,timeUnits::msec);
  forwardTime(200,-10);
  armLeft.setBrake(brakeType::coast);
  armRight.setBrake(brakeType::coast);
  armLeft.setVelocity(-30,percentUnits::pct);
  armRight.setVelocity(-30,percentUnits::pct);
  // armLeft.spin(directionType::fwd);
  // armRight.spin(directionType::fwd);
  armLeft.stop();
  armRight.stop();

  LeftFront.spinFor(toDeg(-3),rotationUnits::deg,10,velocityUnits::pct,false);
  LeftBack.spinFor(toDeg(-3),rotationUnits::deg,10,velocityUnits::pct,false);
  RightFront.spinFor(toDeg(3),rotationUnits::deg,10,velocityUnits::pct,false);
  RightBack.spinFor(toDeg(-3),rotationUnits::deg,10,velocityUnits::pct,false);
  ramp.setVelocity(38,percentUnits::pct);
  ramp.rotateTo(1250,rotationUnits::deg);
  armLeft.stop();
  armRight.stop();
  wait(200,timeUnits::msec);
  armLeft.setVelocity(-100,percentUnits::pct);
  armRight.setVelocity(-100,percentUnits::pct);
  armLeft.rotateFor(800,rotationUnits::deg,false);
  armRight.rotateFor(800,rotationUnits::deg,false);
  ramp.setVelocity(100,percentUnits::pct);
  ramp.rotateTo(20,rotationUnits::deg,false);
  forwardDistance(-30,40);
  armLeft.stop();
  armRight.stop();

}
void redEight(){
  //---------------Expansion--------------
  //expansion
  armLeft.setVelocity(100,percentUnits::pct);
  armRight.setVelocity(100,percentUnits::pct);
  armLeft.spin(directionType::rev);
  armRight.spin(directionType::rev);
  task::sleep(200);
  armLeft.spin(directionType::fwd);
  armRight.spin(directionType::fwd);
  forwardTime(100,-40);
  // task::sleep(100);
  //end of expansion
  //----------------intake---------------
  armLeft.setVelocity(100,percentUnits::pct);
  armRight.setVelocity(100,percentUnits::pct);
  armLeft.spin(directionType::fwd);//
  armRight.spin(directionType::fwd);

  forwardDistance(87,45);
  task::sleep(150);
  turn(0,120,40);
  task::sleep(150);
  forwardDistance(-90,50);
  task::sleep(150);
  turn(1,128,40);
  task::sleep(150);
  forwardDistance(85,30);
  task::sleep(100);
  forwardDistance(-10,25);
  armLeft.setBrake(hold);
  armRight.setBrake(hold);
  task::sleep(150);
  turn(0,348,30);
  task::sleep(150);

  armLeft.setVelocity(8,percentUnits::pct);
  armRight.setVelocity(8,percentUnits::pct);
  //-------------------Stacking-------------
  forwardTime(1100,95);
  task::sleep(100);


  armLeft.setBrake(brakeType::coast);
  armRight.setBrake(brakeType::coast);
  armLeft.setVelocity(-18,percentUnits::pct);
  armRight.setVelocity(-18,percentUnits::pct);
  armLeft.spin(directionType::fwd);
  armRight.spin(directionType::fwd);
  ramp.setVelocity(50,percentUnits::pct);
  // ramp.rotateTo(700,rotationUnits::deg);
  // armLeft.setVelocity(50,percentUnits::pct);
  // armRight.setVelocity(50,percentUnits::pct);
  // armLeft.rotateFor(500,rotationUnits::deg,false);
  // armRight.rotateFor(500,rotationUnits::deg,false);
  ramp.rotateTo(300,rotationUnits::deg);
  armLeft.setVelocity(50,percentUnits::pct);
  armRight.setVelocity(50,percentUnits::pct);
  armLeft.rotateFor(400,rotationUnits::deg,false);
  armRight.rotateFor(400,rotationUnits::deg,false);
  ramp.rotateTo(700,rotationUnits::deg);
  armLeft.stop(); armRight.stop();
  ramp.setVelocity(25,percentUnits::pct);
  ramp.rotateTo(1200,rotationUnits::deg);
  armLeft.stop();
  armRight.stop();
  // task::sleep(100);
  // forwardTime(300,10);
  task::sleep(200);
  armLeft.setVelocity(-20,percentUnits::pct);
  armRight.setVelocity(-20,percentUnits::pct);
  // armLeft.spin(directionType::fwd);
  // armRight.spin(directionType::fwd);
  armLeft.rotateFor(800,rotationUnits::deg,false);
  armRight.rotateFor(800,rotationUnits::deg,false);
  ramp.setVelocity(100,percentUnits::pct);
  ramp.rotateTo(400,rotationUnits::deg,false);
  forwardDistance(-30,60); // 25
  armLeft.stop();
  armRight.stop();

}
void bluFive() {
  armLeft.setBrake(brakeType::coast);
  armRight.setBrake(brakeType::coast);
  //autonomous skills code. Code for the actual competition is yet to be
  //implemented as here the robot doesn’t start from a base.
  
  //-------------Expansion-------------
  // armLeft.setVelocity(100,percentUnits::pct);
  // armRight.setVelocity(100,percentUnits::pct);
  // armLeft.spin(directionType::rev);
  // armRight.spin(directionType::rev);
  // task::sleep(350);
  // armLeft.spin(directionType::fwd);
  // armRight.spin(directionType::fwd);
  // forwardTime(350,-40);
  // task::sleep(150);
  //--------End of Expansion------------

  armLeft.setVelocity(100,percentUnits::pct);
  armRight.setVelocity(100,percentUnits::pct);
  armLeft.spin(directionType::fwd);//
  armRight.spin(directionType::fwd);
  forwardDistance(110,30);
  task::sleep(500);
  armLeft.rotateFor(-75,rotationUnits::deg,false);
  armRight.rotateFor(-75,rotationUnits::deg,false);

  forwardDistance(-20,30);
  task::sleep(300);
  turn(1,410,20); //389
  task::sleep(300);

  forwardTime(1000,60);
  task::sleep(200);
  //forwardTime(550,-10);

  ramp.setVelocity(45,percentUnits::pct);
  ramp.rotateTo(1300,rotationUnits::deg);
  task::sleep(300);
  forwardTime(500,15);
  task::sleep(500);
  armLeft.setVelocity(-20,percentUnits::pct);
  armRight.setVelocity(-20,percentUnits::pct);
  armLeft.rotateFor(800,rotationUnits::deg,false);
  armRight.rotateFor(800,rotationUnits::deg,false);
  ramp.setVelocity(100,percentUnits::pct);
  ramp.rotateTo(400,rotationUnits::deg,false);
  forwardDistance(-30,60); // 25
  armLeft.stop();
  armRight.stop();
}
void one() {//hopefully never have to use this
  armLeft.setVelocity(100,percentUnits::pct);
  armRight.setVelocity(100,percentUnits::pct);
  armLeft.spin(directionType::rev);
  armRight.spin(directionType::rev);
  task::sleep(500);
  armLeft.spin(directionType::fwd);
  armRight.spin(directionType::fwd);
  forwardTime(300,-40);
  task::sleep(150);
  armLeft.stop();
  armRight.stop();
  forwardTime(5000,30);
  forwardTime(2000,-30);
}
#pragma endregion
#pragma region compTemplate

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  armLift.resetRotation();
  ramp.resetRotation();

}

void timeFunction(void(*f)()) { // this function is untested
  auto start = high_resolution_clock::now();
  (*f)();
  auto stopp = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(stopp - start);
  std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;
}
void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  auto start = high_resolution_clock::now();
  bluFive();
  auto stopp = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(stopp - start);
  std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;
  // timeFunction(&blueBoi);
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  Controller1.ButtonY.pressed(controlMode);
  Controller1.ButtonRight.pressed(intakeMode);
  while (1) {
    arm();
    tank();
    //userAcceleration();
    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
  // auto start = high_resolution_clock::now();
  // bluFive();
  // auto stopp = high_resolution_clock::now();
  // auto duration = duration_cast<milliseconds>(stopp - start);
  // std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(10, msec);
  }
}
#pragma endregion
