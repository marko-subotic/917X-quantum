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

vex::motor Left = vex::motor(vex::PORT1);
vex::motor Right = vex::motor(vex::PORT11, true);

vex::motor IntakeLeft = vex::motor(vex::PORT2, true);
vex::motor IntakeRight = vex::motor(vex::PORT12);

vex::motor ArmAngleL = vex::motor(vex::PORT13, true);
vex::motor ArmAngleR = vex::motor(vex::PORT3);

vex::motor TrayAngle = vex::motor(vex::PORT4, true);

///////////////////////////////////////////////////////////////
//                                                           //
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
static int side = 1; // SET FOR SIDE (1 FOR BLUE, -1 FOR RED)!!
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

// move (in feet; + forward, - backwards)
void move(double dist) {
  dist *= 343.77468;
  Left.startRotateFor(dist, vex::rotationUnits::deg, 40,
                      vex::velocityUnits::pct);
  Right.rotateFor(dist, vex::rotationUnits::deg, 40, vex::velocityUnits::pct);
}

// turn degrees, (+ for right, - for left)
void turn(int deg, int side) {
  deg *= 2.4 * side;
  Left.startRotateFor(deg, vex::rotationUnits::deg, 70,
                      vex::velocityUnits::pct);
  Right.rotateFor(-deg, vex::rotationUnits::deg, 70, vex::velocityUnits::pct);
}

// grab one cube and keep in robot, time in seconds, push for direction (+ for
// intake, - for pushing)
void grab(int time, int push) {
  Left.startRotateFor(130 * time, vex::rotationUnits::deg, 30,
                      vex::velocityUnits::pct);
  Right.startRotateFor(130 * time, vex::rotationUnits::deg, 30,
                       vex::velocityUnits::pct);
  IntakeLeft.spin(vex::directionType::rev, 90 * push, vex::velocityUnits::pct);
  IntakeRight.spin(vex::directionType::rev, 90 * push, vex::velocityUnits::pct);
  vex::task::sleep(time * 1000);
  IntakeLeft.spin(vex::directionType::rev, 50 * push, vex::velocityUnits::rpm);
  IntakeRight.spin(vex::directionType::rev, 50 * push, vex::velocityUnits::rpm);
}

// take out cube, lift arm, place cube, and lower WIP
void tower() {
  IntakeLeft.stop();
  IntakeRight.stop();
  IntakeLeft.startRotateFor(200, vex::rotationUnits::deg, 80,
                            vex::velocityUnits::pct);
  IntakeRight.rotateFor(200, vex::rotationUnits::deg, 80,
                        vex::velocityUnits::pct);
  ArmAngleL.startRotateTo(240, vex::rotationUnits::deg, 30,
                          vex::velocityUnits::pct);
  ArmAngleR.rotateTo(240, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  vex::task::sleep(500);
  IntakeLeft.startRotateFor(180, vex::rotationUnits::deg, 90,
                            vex::velocityUnits::pct);
  IntakeRight.rotateFor(180, vex::rotationUnits::deg, 90,
                        vex::velocityUnits::pct);
  vex::task::sleep(500);
  ArmAngleL.startRotateTo(55, vex::rotationUnits::deg);
  ArmAngleR.rotateTo(55, vex::rotationUnits::deg);
}

// slowly lower tray and score
void stack() {
  vex::task::sleep(500);

  IntakeLeft.startRotateFor(90, vex::rotationUnits::deg, -20,
                            vex::velocityUnits::pct);
  IntakeRight.startRotateFor(90, vex::rotationUnits::deg, -20,
                             vex::velocityUnits::pct);

  Left.startRotateFor(90, vex::rotationUnits::deg, 5, vex::velocityUnits::pct);
  Right.startRotateFor(90, vex::rotationUnits::deg, 5, vex::velocityUnits::pct);

  TrayAngle.rotateFor(380, vex::rotationUnits::deg, 30,
                      vex::velocityUnits::rpm);

  Left.startRotateFor(30, vex::rotationUnits::deg, 10, vex::velocityUnits::pct);
  Right.rotateFor(30, vex::rotationUnits::deg, 10, vex::velocityUnits::pct);

  IntakeLeft.startRotateFor(180, vex::rotationUnits::deg, -20,
                            vex::velocityUnits::pct);
  IntakeRight.startRotateFor(180, vex::rotationUnits::deg, -20,
                             vex::velocityUnits::pct);

  Left.startRotateFor(-360, vex::rotationUnits::deg, 20,
                      vex::velocityUnits::pct);
  Right.rotateFor(-360, vex::rotationUnits::deg, 20, vex::velocityUnits::pct);

  TrayAngle.rotateFor(-400, vex::rotationUnits::deg, 30,
                      vex::velocityUnits::rpm);

  ArmAngleL.startRotateTo(65, vex::rotationUnits::deg);
  ArmAngleR.rotateTo(65, vex::rotationUnits::deg);
}

/////////////////////////////////////////////////////
//                                                 //
//                                                 //
//                     FULL AUTON                  //
//                                                 //
//                                                 //
/////////////////////////////////////////////////////

void onePoint() {
  move(0.5);
  move(-1);
  vex::task::sleep(2000);
  move(2);
}

void threePoint() {
  move(1.7);
  grab(5, 1);
  move(-2.4);
  turn(-91, side);
  move(2.5);
  stack();
  vex::task::sleep(500);
}

void autonomous(void) {

  ArmAngleL.startRotateTo(100, vex::rotationUnits::deg, 30,
                          vex::velocityUnits::pct);
  ArmAngleR.rotateTo(100, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  vex::task::sleep(250);
  ArmAngleL.startRotateTo(30, vex::rotationUnits::deg);
  ArmAngleR.rotateTo(55, vex::rotationUnits::deg);
  vex::task::sleep(1000);

  onePoint();
}

///////////////////////////////////////////////////////
//                                                   //
//                                                   //
//                INTAKE IN & OUT                    //
//                                                   //
//                                                   //
///////////////////////////////////////////////////////

static int intake = 0;

void intakeInFunc() {
  // intakeIn *= -1;
  // intakeOut = -1;
  if (intake != 20) {
    intake = 20;
  } else if (intake == 20) {
    intake = 0;
  }
}

void intakeOutFunc() {
  // *= -1;
  // intakeIn = -1;
  if (intake != -200) {
    intake = -200;
  } else if (intake == -200) {
    intake = 0;
  }
}

///////////////////////////////////////////////////////
//                                                   //
//                                                   //
//               DRIVER CONTROL CODE                 //
//                                                   //
//                                                   //
///////////////////////////////////////////////////////

void usercontrol(void) {
  while (1) {

    ///////////////////////////////////////////////////////
    //                                                   //
    //                                                   //
    //                  DRIVE CODE                       //
    //                                                   //
    //                                                   //
    ///////////////////////////////////////////////////////

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
    if ((Controller1.ButtonUp.pressing())) {
      Left.spin(vex::directionType::fwd, 40, vex::velocityUnits::rpm);
      Right.spin(vex::directionType::fwd, 40, vex::velocityUnits::rpm);
    }else{
    Left.spin(vex::directionType::fwd, LeftSide, vex::velocityUnits::rpm);
    Right.spin(vex::directionType::fwd, RightSide, vex::velocityUnits::rpm);
    }

    ////////////////////////////////////////////////////////
    //                                                    //
    //                                                    //
    //                   INTAKE CONTROL                   //
    //                                                    //
    //                                                    //
    ////////////////////////////////////////////////////////

    Controller1.ButtonR1.pressed(intakeInFunc);
    Controller1.ButtonR2.pressed(intakeOutFunc);

    // if (intakeIn == 1) {
    // IntakeLeft.spin(vex::directionType::fwd, 200, vex::velocityUnits::rpm);
    // IntakeRight.spin(vex::directionType::fwd, 200, vex::velocityUnits::rpm);
    // } else if (intakeOut == 1) {
    // IntakeLeft.spin(vex::directionType::fwd, -100, vex::velocityUnits::rpm);
    // IntakeRight.spin(vex::directionType::fwd, -100, vex::velocityUnits::rpm);
    // } else {
    // IntakeLeft.setBrake(hold);
    // IntakeRight.setBrake(vex::brakeType::hold);
    // }
    IntakeLeft.spin(vex::directionType::fwd, intake, vex::velocityUnits::rpm);
    IntakeRight.spin(vex::directionType::fwd, intake, vex::velocityUnits::rpm);

    ///////////////////////////////////////////////////////////
    //                                                       //
    //                                                       //
    //                      ARM ANGLE                        //
    //                                                       //
    //                                                       //
    ///////////////////////////////////////////////////////////

    if ((Controller1.ButtonL1.pressing())) {
      ArmAngleL.spin(vex::directionType::fwd, 40, vex::velocityUnits::rpm);
      ArmAngleR.spin(vex::directionType::fwd, 40, vex::velocityUnits::rpm);
    }

    else if ((Controller1.ButtonL2.pressing())) {
      ArmAngleL.spin(vex::directionType::rev, 40, vex::velocityUnits::rpm);
      ArmAngleR.spin(vex::directionType::rev, 40, vex::velocityUnits::rpm);
    }

    else {
      ArmAngleL.stop(vex::brakeType::hold);
      ArmAngleR.stop(vex::brakeType::hold);
    }

    /*if (Controller1.ButtonUp.pressing()) {
      ArmAngleL.startRotateTo(210, vex::rotationUnits::deg);
      ArmAngleR.rotateTo(210, vex::rotationUnits::deg);
    }

    if (Controller1.ButtonDown.pressing()) {
      ArmAngleL.startRotateTo(55, vex::rotationUnits::deg);
      ArmAngleR.rotateTo(55, vex::rotationUnits::deg);
    }*/

    //////////////////////////////////////////////////////////
    //                                                      //
    //                                                      //
    //                   TRAY ANGLE                         //
    //                                                      //
    //                                                      //
    //////////////////////////////////////////////////////////

    if (Controller1.ButtonA.pressing()) {
      TrayAngle.spin(vex::directionType::fwd, 90, vex::velocityUnits::rpm);

    } else if (Controller1.ButtonB.pressing()) {

      TrayAngle.spin(vex::directionType::rev, 90, vex::velocityUnits::rpm);

    } else {

      TrayAngle.stop(vex::brakeType::hold);
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
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
