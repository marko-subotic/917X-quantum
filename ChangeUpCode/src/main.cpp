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
// Inert                inertial      5               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// distan               encoder       A, B            
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
void move(double dist, int spd)
{
    dist *=(343.77468*2);
    LeftBack.startRotateFor(dist, vex::rotationUnits::deg, spd, vex::velocityUnits::pct);
    LeftFront.startRotateFor(dist, vex::rotationUnits::deg, spd, vex::velocityUnits::pct);

    RightBack.startRotateFor(dist, vex::rotationUnits::deg, spd, vex::velocityUnits::pct);
    RightFront.rotateFor(dist, vex::rotationUnits::deg, spd, vex::velocityUnits::pct);
}
void moveForward(double dist, int spd)
{
  distan.setPosition(0,degrees);
  bool goingUp = true;
  double firstAngle = inert.orientation(roll,degrees);
  double currentAngle = inert.orientation(roll,degrees);
  double error = dist;
  double kError = 1;
  double kChange = 1;
  double leftSpeed = spd;
  double rightSpeed = spd;
  int range = 1;
  dist *=(343.77468*2);
  while(error>=5){
    currentAngle = inert.orientation(roll,degrees);
    if(firstAngle-currentAngle!=0){
      if(currentAngle<firstAngle){
        if(goingUp){
          leftSpeed += (currentAngle-firstAngle)*kChange;
        }else{
          rightSpeed -= (currentAngle-firstAngle)*kChange;
        }
        if(leftSpeed-spd>range){
          goingUp = false;
        }else if(leftSpeed-spd<range){
          goingUp = true;
        }
        if(rightSpeed-spd>range){
          goingUp = false;
        }else if(rightSpeed-spd<range){
          goingUp = true;
        }
      }
      if(currentAngle>firstAngle){
        if(goingUp){
          rightSpeed += (currentAngle-firstAngle)*kChange;
        }else{
          leftSpeed -= (currentAngle-firstAngle)*kChange;
        }
        if(leftSpeed-spd>range){
          goingUp = false;
        }else if(leftSpeed-spd<range){
          goingUp = true;
        }
        if(rightSpeed-spd>range){
          goingUp = false;
        }else if(rightSpeed-spd<range){
          goingUp = true;
        }
      }
    }
    leftDrive.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    rightDrive.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    error -= (distan.position(degrees)*M_PI*4.1);
  }
  while(error>=.01){
    leftDrive.spin(vex::directionType::fwd,kError*(error),vex::velocityUnits::pct);
    rightDrive.spin(vex::directionType::fwd,kError*(error),vex::velocityUnits::pct);
    error -= (distan.position(degrees)*M_PI*4.1);

  }
}
void accelerate(double dist, int spd)
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
void grab(double time, int push)
{
    LeftFront.startRotateFor(310*time, vex::rotationUnits::deg, 31, vex::velocityUnits::pct);
    LeftBack.startRotateFor(310*time, vex::rotationUnits::deg, 31, vex::velocityUnits::pct);

    RightFront.startRotateFor(310*time, vex::rotationUnits::deg, 31, vex::velocityUnits::pct);
    RightBack.startRotateFor(310*time, vex::rotationUnits::deg, 31, vex::velocityUnits::pct);
    
    /*
    LeftFront.spin(vex::directionType::fwd, 35, vex::velocityUnits::pct); 
    RightFront.spin(vex::directionType::fwd, 35, vex::velocityUnits::pct);

    LeftBack.spin(vex::directionType::fwd, 35, vex::velocityUnits::pct); 
    RightBack.spin(vex::directionType::fwd, 35, vex::velocityUnits::pct);
    */

    IntakeLeft.spin(vex::directionType::rev, 99*push, vex::velocityUnits::pct); 
    IntakeRight.spin(vex::directionType::rev, 99*push, vex::velocityUnits::pct);
    vex::task::sleep(time*1000);

    LeftFront.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct); 
    RightFront.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);

    LeftBack.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct); 
    RightBack.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
    vex::task::sleep(400);

    IntakeLeft.spin(vex::directionType::rev, 0, vex::velocityUnits::rpm); 
    IntakeRight.spin(vex::directionType::rev, 0, vex::velocityUnits::rpm); 
}

//take out cube, lift arm, place cube, and lower WIP
/*void tower()
{
    IntakeLeft.stop(); 
    IntakeRight.stop();
    
    IntakeLeft.startRotateFor(200, vex::rotationUnits::deg, 80, vex::velocityUnits::pct);
    IntakeRight.rotateFor(200, vex::rotationUnits::deg, 80, vex::velocityUnits::pct); 
    
    TrayAngle.rotateFor(40, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);

    ArmAngle.rotateFor(240, vex::rotationUnits::deg, 30, vex::velocityUnits::pct); 
    vex::task::sleep(500);
    IntakeLeft.startRotateFor(180, vex::rotationUnits::deg, 90, vex::velocityUnits::pct);
    IntakeRight.rotateFor(180, vex::rotationUnits::deg, 90, vex::velocityUnits::pct); 
    vex::task::sleep(500);
    ArmAngle.rotateTo(55, vex::rotationUnits::deg); 
}

//slowly lower tray and score
void stack()
{
    IntakeLeft.startRotateFor(360, vex::rotationUnits::deg, -35, vex::velocityUnits::pct);
    IntakeRight.startRotateFor(360, vex::rotationUnits::deg, -35, vex::velocityUnits::pct);

    TrayAngle.rotateFor(-1100, vex::rotationUnits::deg, 70, vex::velocityUnits::pct);
    TrayAngle.rotateFor(-200, vex::rotationUnits::deg, 25, vex::velocityUnits::pct);
    vex::task::sleep(200);

    LeftFront.startRotateFor(35, vex::rotationUnits::deg, 20, vex::velocityUnits::pct);
    LeftBack.startRotateFor(35, vex::rotationUnits::deg, 20, vex::velocityUnits::pct);
    RightFront.startRotateFor(35, vex::rotationUnits::deg, 20, vex::velocityUnits::pct);
    RightBack.rotateFor(35, vex::rotationUnits::deg, 20, vex::velocityUnits::pct);
    vex::task::sleep(200);

    LeftFront.startRotateFor(-720, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
    LeftBack.startRotateFor(-720, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
    RightFront.startRotateFor(-720, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
    RightBack.rotateFor(-720, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
}

//stuff before auton
void start()
{
    //side=s;
    TrayAngle.rotateTo(-500, vex::rotationUnits::deg, 80, vex::velocityUnits::pct);
    vex::task::sleep(250);
    ArmAngle.startRotateTo(-700, vex::rotationUnits::deg, 60, vex::velocityUnits::pct);
    vex::task::sleep(300);

    TrayAngle.rotateTo(-350, vex::rotationUnits::deg, 40, vex::velocityUnits::pct);
    vex::task::sleep(100);
    ArmAngle.rotateTo(-69, vex::rotationUnits::deg);
    vex::task::sleep(400);
}



/////////////////////////////////////////////////////
//                                                 //
//                                                 //
//                     FULL AUTON                  //
//                                                 //
//                                                 //
/////////////////////////////////////////////////////


void threePoint() {
  move(1.7);
  grab(5, 1);
  move(-2.4);
  turn(-91, side);
  move(2.5);
  stack();
  vex::task::sleep(500);
}


//overall speed
int s=35;

void one() {
  //start();
    vex::task::sleep(500);

    //start bot 1 tile from goal zone, facing backwards
    move(-1,s);
    move(2,s+10);

}

void five() 
{
    start();
    
    move(0.1,s);
    grab(4.05,1);
    vex::task::sleep(100);

    move(-1.06,s+12);
    turn(-135);
    vex::task::sleep(400);
    move(0.67,s);

    stack();
}

void autonomous(void) {
  one();
}*/

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
    topRoller = 100;
  } else if (bottomRoller == 100) {
    bottomRoller = 0;
    topRoller = 0;
  }
  Brain.Screen.clearLine();
  Brain.Screen.print("roller moving in\v");
}

void rollerOutFunc() {
  if (bottomRoller != -100) {
    bottomRoller = -100;
    topRoller = -100;
  } else if (bottomRoller == -100) {
    bottomRoller = 0;
    topRoller = 0;
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
  Controller1.ButtonX.pressed(rollerOutFunc);

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
    if ((Controller1.ButtonUp.pressing())) {
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
  //Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
