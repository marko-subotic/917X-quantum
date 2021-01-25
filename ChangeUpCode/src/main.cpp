// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// distan               encoder       G, H            
// inert                inertial      5               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// distan               encoder       G, H            
// inert                inertial      5               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// distan               encoder       A, B            
// inert                inertial      5               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// distan               encoder       A, B            
// inertial             inertial      5               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// distan               encoder       A, B            
// Inertial             inertial      5               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// distan               encoder       A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// dist                 encoder       A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// dist                 encoder       A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// EncoderA             encoder       A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;

// A global instance of competition
competition Competition;

// Declares all motors and the controller

vex::controller Controller1 = vex::controller();
vex::controller Controller2 = vex::controller();

vex::motor LeftBack = vex::motor(vex::PORT19);
vex::motor RightBack = vex::motor(vex::PORT20, true);
vex::motor LeftFront = vex::motor(vex::PORT17);
vex::motor RightFront = vex::motor(vex::PORT18,true);
motor_group   leftDrive( LeftBack, LeftFront);
motor_group   rightDrive( RightBack, RightFront);
//vex::encoder dist = vex::encoder(vex::PORTA,vex::PORTB)

vex::motor IntakeLeft = vex::motor(vex::PORT3,true);
vex::motor IntakeRight = vex::motor(vex::PORT4);

vex::motor BottomRoller = vex::motor(vex::PORT1);

vex::motor TopRoller = vex::motor(vex::PORT2, ratio6_1);

///////////////////////////////////////////////////////////////
//                                                           //
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//blue: 1, red: -1
int side=1;
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//                                                           //
///////////////////////////////////////////////////////////////

// Runs preauton sequnces
void pre_auton(void) { vexcodeInit(); }

///////////////////////////////////////////////////////////////
//                                                           //
//                                                           //
//                AUTON METHODS                              //
//                                                           //
//                                                           //
///////////////////////////////////////////////////////////////

//move (in tiles; + forward, - backwards)

const double EncoderWheelDiameterInches = 4.37;
const double DistanceUntilDecelerateInches = 20;
const double DistanceUntilAccelerate = 5;
const double DistanceUntilLinearInches = 3;
const double LinearSpeed = 5;

const double MinErrorInches = .01;

void encoderTest(){
  distan.resetRotation();
  while(true){
    Brain.Screen.clearLine();
    Brain.Screen.print(distan.position(degrees));

  }
}
void moveForward(double dist, int spd)
{
  int oSpeed = spd;
  spd = LinearSpeed;
  distan.resetRotation();
  double firstAngle = inert.orientation(roll,degrees);
  double currentAngle = inert.orientation(roll,degrees);
  double error = dist;
  double kError = 1/DistanceUntilDecelerateInches;
  double kChange = 1.2;
  double kAccel = (oSpeed/(LinearSpeed*DistanceUntilAccelerate))-(1/DistanceUntilAccelerate);
  double leftSpeed = spd;
  double rightSpeed = spd;

  //Brain.Screen.clearLine();
   // Brain.Screen.print("distance till: ");
    //Brain.Screen.print(error);
  while(error>=MinErrorInches){
    double distanceCovered =  ((distan.position(degrees)/360)*M_PI*EncoderWheelDiameterInches);
    if(distanceCovered<=DistanceUntilAccelerate){
      spd = LinearSpeed*(1 + distanceCovered * kAccel);
    }else if(error<=DistanceUntilDecelerateInches){
      spd = oSpeed*error*kError;
      if(error<=DistanceUntilLinearInches){
        spd = LinearSpeed;
      }
    }
    currentAngle = inert.orientation(yaw,degrees);
    leftSpeed = spd -(currentAngle-firstAngle)*kChange;
    rightSpeed = spd + (currentAngle-firstAngle)*kChange;
    
    LeftBack.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    error = dist - ((distan.position(degrees)/360)*M_PI*EncoderWheelDiameterInches);
    Brain.Screen.clearLine();
    Brain.Screen.print("distance till: ");
    Brain.Screen.print(error);

  }
  leftDrive.stop();
  rightDrive.stop();
  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);
}
void moveBackward(double dist, int spd)
{
  double encoderPlaceholder = 0;
  double error = dist;
  double constant = 1;
  dist *=(343.77468*2);
  while(error>=dist-5){
    leftDrive.spin(vex::directionType::fwd,constant*(error),vex::velocityUnits::pct);
    rightDrive.spin(vex::directionType::fwd,constant*(error),vex::velocityUnits::pct);
    error = error-encoderPlaceholder;
  }
  while(error>=.1){
    leftDrive.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
    rightDrive.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
    error = error-encoderPlaceholder;
  }
  leftDrive.spin(vex::directionType::fwd,0,vex::velocityUnits::pct);
  rightDrive.spin(vex::directionType::fwd,0,vex::velocityUnits::pct);
}
//turn degrees, (+ for right, - for left)
void turn(int deg)
{
    //move(-0.5,60);
    deg *= 2.32*side;
    LeftFront.startRotateFor(deg, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
    LeftBack.startRotateFor(deg, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);

    RightFront.startRotateFor(-deg, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
    RightBack.rotateFor(-deg, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
}

//grab one cube and keep in robot, time in seconds, push for direction (+ for intake, - for pushing)





/////////////////////////////////////////////////////
//                                                 //
//                                                 //
//                     FULL AUTON                  //
//                                                 //
//                                                 //
/////////////////////////////////////////////////////

void autonomous(void) {
  inert.calibrate();
  wait(2000,msec);
  Brain.Screen.print("Pressed");
  moveForward(30,75);
  //encoderTest();
    Brain.Screen.print("Passed");

}

///////////////////////////////////////////////////////
//                                                   //
//                                                   //
//           INTAKE AND ROLLERS IN & OUT             //
//                                                   //
//                                                   //
///////////////////////////////////////////////////////

static int takein = 0;
static int bottomRoller = 0;
static int topRoller = 0;

void topRollerInFunc() {
  if (topRoller != 100) {
    topRoller = 100;
  } else if (topRoller == 100) {
    topRoller = 0;
  }
  Brain.Screen.clearLine();
  Brain.Screen.print("top roller moving in\v");
}

void rollerInFunc() {
  if (bottomRoller != 100) {
    bottomRoller = 100;
  } else if (bottomRoller == 100) {
    bottomRoller = 0;
  }
  Brain.Screen.clearLine();
  Brain.Screen.print("roller moving in\v");
}

void rollerOutFunc() {
  if (bottomRoller != -100) {
    bottomRoller = -100;
  } else if (bottomRoller == -100) {
    bottomRoller = 0;
  }
  Brain.Screen.clearLine();
  Brain.Screen.print("roller moving out\v");
}

void intakeInFunc() {
  if (takein != 100) {
    takein = 100;
  } else if (takein == 100) {
    takein = 0;
  }
  Brain.Screen.clearLine();
  Brain.Screen.print("roller moving in\v");
}

void intakeOutFunc() {
  if (takein != -100) {
    takein = -100;
  } else if (takein == -100) {
    takein = 0;
  }
  Brain.Screen.clearLine();
  Brain.Screen.print("roller moving out\v");
}

///////////////////////////////////////////////////////
//                                                   //
//                                                   //
//               DRIVER CONTROL CODE                 //
//                                                   //
//                                                   //
///////////////////////////////////////////////////////

void usercontrol(void) {
  

  Controller1.ButtonL1.pressed(rollerInFunc);
  Controller1.ButtonL2.pressed(rollerOutFunc);
  Controller1.ButtonX.pressed(topRollerInFunc);
  Controller1.ButtonA.pressed(autonomous);

  Controller1.ButtonR1.pressed(intakeInFunc);
  Controller1.ButtonR2.pressed(intakeOutFunc);
  while (1) {

    ///////////////////////////////////////////////////////
    //                                                   //
    //                                                   //
    //                  DRIVE CODE                       //
    //                                                   //
    //                                                   //
    ///////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////
    //                                                    //
    //                                                    //
    //               INTAKE & ROLLER CONTROL              //
    //                                                    //
    //                                                    //
    ////////////////////////////////////////////////////////

    

    //Controller1.ButtonL1.pressed(rollerInFunc);
    //Controller1.ButtonL2.pressed(rollerOutFunc);

    
    IntakeLeft.spin(vex::directionType::fwd, takein, vex::velocityUnits::pct);
    IntakeRight.spin(vex::directionType::fwd, takein, vex::velocityUnits::pct);

    BottomRoller.spin(vex::directionType::fwd, -bottomRoller, vex::velocityUnits::pct);
    TopRoller.spin(vex::directionType::fwd, topRoller, vex::velocityUnits::pct);
    

    int LeftSide1 = Controller1.Axis3.value();
    int RightSide1 = Controller1.Axis2.value();

    int LeftSide2 = Controller2.Axis3.value();
    int RightSide2 = Controller2.Axis2.value();

    int LeftSide;
    int RightSide;

    LeftSide = (LeftSide1 * LeftSide1 * LeftSide1) / (16629);

    if ((abs(LeftSide1) <= 10) and (abs(LeftSide2) >= 10)) {
      LeftSide = (LeftSide2 * LeftSide2 * LeftSide2) / (16629);
    }

    RightSide = (RightSide1 * RightSide1 * RightSide1) / (16629);

    if ((abs(RightSide1) <= 10) and (abs(RightSide2) >= 10)) {
      RightSide = (RightSide2 * RightSide2 * RightSide2) / (16629);
    }
    if ((Controller1.ButtonRight.pressing())) {
      BottomRoller.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct);

    }else if ((Controller1.ButtonDown.pressing())) {
      BottomRoller.spin(vex::directionType::fwd, -40, vex::velocityUnits::pct);
      
    }
    else{
    LeftBack.spin(vex::directionType::fwd, LeftSide, vex::velocityUnits::rpm);
    LeftFront.spin(vex::directionType::fwd, LeftSide, vex::velocityUnits::rpm);

    RightBack.spin(vex::directionType::fwd, RightSide, vex::velocityUnits::rpm);
    RightFront.spin(vex::directionType::fwd, RightSide, vex::velocityUnits::rpm);

    }

    
    //////////////////////////////////////////////////////////
    //                                                      //
    //                                                      //
    //                       STACK BUTTON                   //
    //                                                      //
    //                                                      //
    //////////////////////////////////////////////////////////

    //Controller1.ButtonX.pressed(stack);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//////////////////////////////////////////////////////////////
//                                                          //
//                                                          //
//                      MAIN                                //
//                                                          //
//                                                          //
//////////////////////////////////////////////////////////////

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
