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

vex::motor IntakeLeft = vex::motor(vex::PORT3, true);
vex::motor IntakeRight = vex::motor(vex::PORT4);

vex::motor BottomRoller = vex::motor(vex::PORT1, true);

vex::motor TopRoller = vex::motor(vex::PORT2, true);

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
static int roller = 0;





void rollerInFunc() {
  if (roller != 200) {
    roller = 200;
  } else if (roller == 200) {
    roller = 0;
  }
  Brain.Screen.clearLine();
  Brain.Screen.print("roller moving in\v");
}

void rollerOutFunc() {
  if (roller != -400) {
    roller = -400;
  } else if (roller == -400) {
    roller = 0;
  }
  Brain.Screen.clearLine();
  Brain.Screen.print("roller moving out\v");
}

void intakeInFunc() {
  if (takein != 200) {
    takein = 200;
  } else if (takein == 200) {
    takein = 0;
  }
  Brain.Screen.clearLine();
  Brain.Screen.print("roller moving in\v");
}

void intakeOutFunc() {
  if (takein != -400) {
    takein = -400;
  } else if (takein == -400) {
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

    Controller1.ButtonR1.pressed(intakeInFunc);
    Controller1.ButtonR2.pressed(intakeOutFunc);

    Controller1.ButtonL1.pressed(rollerInFunc);
    Controller1.ButtonL2.pressed(rollerOutFunc);

    
    IntakeLeft.spin(vex::directionType::fwd, takein, vex::velocityUnits::rpm);
    IntakeRight.spin(vex::directionType::fwd, takein, vex::velocityUnits::rpm);

    BottomRoller.spin(vex::directionType::fwd, -roller, vex::velocityUnits::rpm);
    TopRoller.spin(vex::directionType::fwd, roller, vex::velocityUnits::rpm);
    

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
      LeftBack.spin(vex::directionType::fwd, 40, vex::velocityUnits::rpm);
      LeftFront.spin(vex::directionType::fwd, 40, vex::velocityUnits::rpm);

      RightBack.spin(vex::directionType::fwd, 40, vex::velocityUnits::rpm);
      RightFront.spin(vex::directionType::fwd, 40, vex::velocityUnits::rpm);

    }else if ((Controller1.ButtonDown.pressing())) {
      LeftBack.spin(vex::directionType::fwd, -80, vex::velocityUnits::rpm);
      LeftFront.spin(vex::directionType::fwd, -80, vex::velocityUnits::rpm);

      RightBack.spin(vex::directionType::fwd, -80, vex::velocityUnits::rpm);
      RightFront.spin(vex::directionType::fwd, -80, vex::velocityUnits::rpm);

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
