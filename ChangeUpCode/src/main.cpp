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
void pre_auton(void) { 
  vexcodeInit(); 
  
  }

///////////////////////////////////////////////////////////////
//                                                           //
//                                                           //
//                AUTON METHODS                              //
//                                                           //
//                                                           //
///////////////////////////////////////////////////////////////

//move (in tiles; + forward, - backwards)



void encoderTest(){
  distan.resetRotation();
  while(true){
    Brain.Screen.clearLine();

    Brain.Screen.print("Pitch: ");
    Brain.Screen.print(inert.orientation(pitch,degrees));
    Brain.Screen.print(" Roll: ");
    Brain.Screen.print(inert.orientation(roll,degrees));
    Brain.Screen.print(" Yaw: ");
    Brain.Screen.print(inert.orientation(yaw,degrees));

  }
}

bool turnRight(int ang, double currentAng){
  if(ang-currentAng>=0){
    if(ang-currentAng>=180){
      return false;
    }
    return true;
  }else{
    if(ang-currentAng<=-180){
      return true;
    }
    return false;
  }
}
const double AngleUntilAccelerate = 5;
const double TurnLinearSpeed = 5;
const double MinErrorDegrees = .3;

void turn(double ang, int spd)
{



  Brain.Screen.print(inert.orientation(yaw,degrees));
  Brain.Screen.newLine();
  int oSpeed = spd;
  spd = TurnLinearSpeed;
  double firstAngle = inert.orientation(yaw,degrees);
  double currentAngle = inert.orientation(yaw,degrees);
  double error =fabs(ang-currentAngle);
  const double AngleUntilDecelerate = fabs(error/2.2);
const double AngleUntilLinear = fabs(error/6);
  double kError = 1/AngleUntilDecelerate;
  double kSlow = 1;
  double kAccel = (oSpeed/(TurnLinearSpeed*AngleUntilAccelerate))-(1/AngleUntilAccelerate);
  bool turnWhere = turnRight(ang,currentAngle);
  

  //Brain.Screen.clearLine();
   // Brain.Screen.print("Angle till: ");
    //Brain.Screen.print(error);
  while(fabs(error)>=MinErrorDegrees){
    Brain.Screen.clearLine();
    
    double distanceCovered =  fabs(inert.orientation(yaw,degrees)-firstAngle);
    if(fabs(error)<=AngleUntilDecelerate){
      spd = oSpeed*fabs(error)*kError*kSlow;
      if(fabs(error)<=AngleUntilLinear){
        spd = TurnLinearSpeed;
      }
    }else if(fabs(distanceCovered)<=AngleUntilAccelerate){
          Brain.Screen.print(fabs(distanceCovered));

      spd = TurnLinearSpeed*(1 + fabs(distanceCovered) * kAccel);
    }
    double leftSpeed = spd;
    double rightSpeed = spd;
    if(turnWhere){
      rightSpeed *= -1;
    }else{
      leftSpeed *= -1;
    }
    currentAngle = inert.orientation(yaw,degrees);    
    LeftBack.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    error = ang - (inert.orientation(yaw,degrees));
    //Brain.Screen.clearLine();
    //Brain.Screen.print("distance till: ");
    //Brain.Screen.print(error);

  }

  leftDrive.stop();
  rightDrive.stop();
  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);
  Brain.Screen.print(inert.orientation(yaw,degrees));
  Brain.Screen.newLine();
}

void move(double dist, double spd,int ang, int angSpeed)
{
const double EncoderWheelDiameterInches = 4.37;
const double DistanceUntilDecelerateInches = 20;
const double DistanceUntilAccelerate = 7;
const double DistanceUntilLinearInches = 4;
const double AngleForMaxError = 60;
const double LinearSpeed = 5;
const double MinErrorInches = .1;
  Brain.Screen.print(inert.orientation(yaw,degrees));
  Brain.Screen.newLine();
  double oSpeed = spd;
  spd = LinearSpeed;
  distan.resetRotation();
  double firstAngle = inert.orientation(yaw,degrees);
  double currentAngle = inert.orientation(yaw,degrees);
  double error = dist;
  double kError = 1/DistanceUntilDecelerateInches;
  double kAccel = (oSpeed/(LinearSpeed*DistanceUntilAccelerate))-(1/DistanceUntilAccelerate);
  double kDecel = (oSpeed-LinearSpeed)/(oSpeed*DistanceUntilDecelerateInches);
  double leftSpeed = spd;
  double rightSpeed = spd;

  //Brain.Screen.clearLine();
   // Brain.Screen.print("distance till: ");
    //Brain.Screen.print(error);
  while(fabs(error)>=MinErrorInches){
    double distanceCovered =  ((distan.position(degrees)/360)*M_PI*EncoderWheelDiameterInches);
    if(fabs(distanceCovered)<=DistanceUntilAccelerate){
      spd = LinearSpeed*(1 + fabs(distanceCovered) * kAccel);
    }else if(fabs(error)<=DistanceUntilDecelerateInches){
      double distanceDecelerated = DistanceUntilDecelerateInches-error;
      spd = oSpeed*(1-fabs(distanceDecelerated)*kDecel);
      //if(fabs(error)<=DistanceUntilLinearInches){
        //spd = LinearSpeed;
      //}
    }else{
      spd = oSpeed;
    }
    currentAngle = inert.orientation(yaw,degrees);
    double deltaTheta = currentAngle-firstAngle;
    double speedCorrection = (spd/AngleForMaxError)*deltaTheta;
    leftSpeed = spd -speedCorrection;
    rightSpeed = spd + speedCorrection;
    if(dist<0){
      double temp = leftSpeed;
      leftSpeed = -rightSpeed;
      rightSpeed = -temp;
    }

          printf("%f\n", spd);

    
    LeftBack.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    error = dist - ((distan.position(degrees)/360)*M_PI*EncoderWheelDiameterInches);
    //Brain.Screen.clearLine();
    //Brain.Screen.print("distance till: ");
    //Brain.Screen.print(error);

  }

  leftDrive.stop();
  rightDrive.stop();
  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);
  Brain.Screen.print(inert.orientation(yaw,degrees));
  Brain.Screen.newLine();
  turn(ang,angSpeed);
}

void intakeL(directionType dir, double time, double velocity){
  IntakeLeft.spinFor(dir, time, sec, -velocity, vex::velocityUnits::pct);
}
void intakeR(directionType dir, double time, double velocity){
  IntakeRight.spinFor(dir, time, sec, -velocity, vex::velocityUnits::pct);
}
void roller(double time, double velocity){
  BottomRoller.spinFor(fwd, time, sec, -velocity, vex::velocityUnits::pct);
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
  
  //TopRoller.spin(fwd,100,pct);
  inert.calibrate();
  Brain.Screen.print("calibrated");
  wait(2000,msec);
  move(50,80,0,20);
  /*Brain.Screen.print("Pressed");
  vex::thread([](){
    intakeL(fwd,60,100);
  }).detach();
  vex::thread([](){
    intakeR(fwd,60,100);
  }).detach();
  move(28,75,-135,45);
  wait(.2,sec);
  move(29.25,75,-135,2);
  roller(.4,100);
  move(-12,30,0,45);
  wait(.2,sec);
  move(36,75,-90,45);
  //turn(-90,22);
  move(4.75,30,-90,2);
  roller(.5,100);
  wait(.25,sec);
  roller(.5,100);*/
  /*move(-15,30,-6,22);
  move(47,70,-52,12);
  move(21,40,-59,10);
  roller(1,100);
  */




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
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
