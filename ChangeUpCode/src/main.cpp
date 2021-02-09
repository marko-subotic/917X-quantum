// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// distanR              encoder       G, H            
// inert                inertial      5               
// distanL              encoder       E, F            
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// distanR              encoder       G, H            
// inert                inertial      5               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// distanR              encoder       A, B            
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
#include <Point.h>
#include "..\..\Shared\GeometryFunctions.cpp"

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
  distanR.resetRotation();
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
double calcAngNeeded(int ang, double currentAng){
  double rtrn = ang-currentAng;
  if(rtrn>=180){
    rtrn-=360;
  } else if(rtrn<=-180){
    rtrn += 360;
  }
  return (rtrn);
}
//const double TurnLinearSpeed = 1;

const double MinErrorDegrees = 1;
static const int printerSize = 10000;
static const int numberIterations = 76941;
static double printer[printerSize][4];
static int printerSamplingRate = numberIterations/printerSize;


void turn(double ang, double spd)
{
  printf("Start: %f\n", inert.orientation(yaw,degrees));

  Brain.Screen.print(inert.orientation(yaw,degrees));
  Brain.Screen.newLine();
  double oSpeed = spd;
  double finalSpeed = 2;
  double initialSpeed = fmin(15,spd);
  spd = initialSpeed;
  double firstAngle = inert.orientation(yaw,degrees);
  double currentAngle = inert.orientation(yaw,degrees);
  double error =fabs(ang-currentAngle);
  double switcher = ang-currentAngle;
  double angNeeded = fabs(calcAngNeeded(ang,currentAngle));
  const double AngleUntilDecelerate = 65.5;
  double AngleUntilAccelerate = 10;
// double AngleUntilLinear = fabs(error/5);
 double NonMaxSpeedDist = AngleUntilDecelerate+AngleUntilAccelerate;

  //double kError = 1/AngleUntilDecelerate;
  //double kSlow = 1;
  double kAccel = (oSpeed/(initialSpeed*AngleUntilAccelerate))-(1/AngleUntilAccelerate);
  double kDecel = (oSpeed-finalSpeed)/(oSpeed*AngleUntilDecelerate);
  bool turnWhere = turnRight(ang,currentAngle);
  double leftSpeed = spd;
  double rightSpeed = spd;

  double distanceCovered =  fabs(inert.orientation(yaw,degrees)-firstAngle);
  int i = 0;
  int j = 0;
  while(fabs(distanceCovered)<=angNeeded - MinErrorDegrees){
    if(switcher>180||switcher<-180){
      if(firstAngle*inert.orientation(yaw,degrees)<0){
        if(inert.orientation(yaw,degrees)>0){
          distanceCovered = fabs(inert.orientation(yaw,degrees)-360-firstAngle);
        }else{
          distanceCovered = fabs(inert.orientation(yaw,degrees)+360-firstAngle);
        }
      }else{
        distanceCovered = fabs(inert.orientation(yaw,degrees)-firstAngle);
      }
    }else{
      distanceCovered = fabs(inert.orientation(yaw,degrees)-firstAngle);
    }
    bool isAccel = false;
    bool isDecel = false;
    if(fabs(angNeeded)<NonMaxSpeedDist){
      if(fabs(distanceCovered) < fabs(angNeeded)*(AngleUntilAccelerate/(NonMaxSpeedDist))){
        isAccel = true;
      }else{
        isDecel = true;
      }
    }else{
      if(fabs(distanceCovered)<=AngleUntilAccelerate){
        isAccel = true;
      }else if(fabs(error)<=AngleUntilDecelerate){
        isDecel = true;
      }  
    }
    if(isAccel){
      spd = initialSpeed*(1 + fabs(distanceCovered) * kAccel);
    }else if(isDecel){
      double distanceDecelerated = AngleUntilDecelerate-fabs(error);
      spd = oSpeed*(1-fabs(distanceDecelerated)*kDecel);
    }else{
      spd = oSpeed;
    }
    leftSpeed = spd;
    rightSpeed = spd;
    if(turnWhere){
      rightSpeed *= -1;
    }else{
      leftSpeed *= -1;
    }
    if(j<printerSize&&i%printerSamplingRate==0){
      printer[j][0] = distanceCovered;
      printer[j][1] = spd;
            j++;
    }
    i ++;
    currentAngle = inert.orientation(yaw,degrees);    
    LeftBack.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    error = ang - (inert.orientation(yaw,degrees));

  }
  
  leftDrive.stop(hold);
  rightDrive.stop(hold);
  LeftFront.setBrake(hold);
  RightFront.setBrake(hold);
  LeftBack.setBrake(hold);
  RightBack.setBrake(hold);
  
  Brain.Screen.print(inert.orientation(yaw,degrees));
  Brain.Screen.newLine();
  printf("%d\n", i);
  /*or(int l = 0; l<printerSize;l++){
    if(printer[l][0]!=0){
      printf("%f\n", printer[l][0]);
      fflush(stdout);
    }
   

  }
  printf("Encoder value: \n");
  for(int l = 0; l<printerSize;l++){
    if(printer[l][1]!=0){
      printf("%f\n", printer[l][1]);
      fflush(stdout);
    }
  }
  */
  //printf("%f\n", inert.orientation(yaw,degrees));
  //wait(1,sec);
  //printf("%f\n", inert.orientation(yaw,degrees));
  
}
/*const double EncoderDist = 14;
typedef struct _Point{
  double x;
  double y;
} Point;
Point calculateNewPoint(Point oldPoint, double alpha, double theta, double radius, int coefficient){
  Point p2;
  Point rotationCenter;
  rotationCenter.x = oldPoint.x-(coefficient*(radius*cos(alpha)));
  rotationCenter.y = oldPoint.y-(coefficient*(radius*sin(alpha)));
  double beta = alpha + theta;
  p2.x = oldPoint.x- coefficient*(oldPoint.x-(coefficient*(cos(beta)*radius)));
  p2.y = oldPoint.y- coefficient*(oldPoint.y-(coefficient*(sin(beta)*radius)));
  return p2;
}
void calculateNewPos(Point leftWheel, Point rightWheel, double leftDist, double rightDist,Point* leftWheelResult, Point* rightWheelResult){
  if(leftDist==rightDist){
    leftWheelResult->x = leftWheel.x;
    leftWheelResult->y = leftWheel.y + leftDist;
    rightWheelResult->x = rightWheel.x;
    rightWheelResult->y = rightWheel.y + leftDist;
    return;
  }
  int coefficient = 1;
  double slope = (rightWheel.y-leftWheel.y)/(rightWheel.x-leftWheel.x);
  double leftRad,rightRad,theta;
  if(rightDist>leftDist){
    leftRad = (leftDist*EncoderDist)/(rightDist-leftDist);
    rightRad = leftRad+EncoderDist;
    theta = leftRad/leftDist;
  }else{
    coefficient *= -1;
    rightRad = (rightDist*EncoderDist)/(leftDist-rightDist);
    leftRad = rightRad+EncoderDist;
    theta = leftRad/leftDist;
  }
  *leftWheelResult = calculateNewPoint(leftWheel,atan(slope),theta,leftRad,coefficient);
  *rightWheelResult = calculateNewPoint(rightWheel,atan(slope),theta,rightRad,coefficient);

  
  
}*/
const double EncoderDist = 13;
void calculateNewPos(Point leftWheel, Point rightWheel, double leftDist, double rightDist,Point* leftWheelResult, Point* rightWheelResult, double EncoderDist);
void move(double dist, double inSpd,double ang, int angSpeed)
{
    double volatile spd = inSpd;
const double EncoderWheelDiameterInches = 4.33;
double DistanceUntilDecelerateInches = 20;
double DistanceUntilAccelerate = 7;
const double DistanceUntilLinearInches = 4;
const double DistanceForMaxError = 750;
//if final is greater than 20 risk of not finishing straight heighens
const double finalSpeedForward = 20;
//if init is great than 35 risk of not going straight heightens
const double initialSpeedForward = 20;
const double finalSpeedBackward = 24;
const double initialSpeedBackward = 18;

double finalSpeed = finalSpeedForward;
double initialSpeed = initialSpeedForward;

if(dist<0){
  finalSpeed = finalSpeedBackward;
  printf("final %f\n", finalSpeed);
  initialSpeed = initialSpeedBackward;
}
const double NonMaxSpeedDist = DistanceUntilDecelerateInches + DistanceUntilLinearInches;
//to give the bot time to slow over small distance
if(fabs(dist)<NonMaxSpeedDist){
  initialSpeed=finalSpeed;
}
  Brain.Screen.print(inert.orientation(yaw,degrees));
  Brain.Screen.newLine();
  double oSpeed = spd;
  spd = initialSpeed;
  distanR.resetRotation();
  distanL.resetRotation();
  double error = dist;
  double kAccel = (oSpeed/(initialSpeed*DistanceUntilAccelerate))-(1/DistanceUntilAccelerate);
  double kDecel = (oSpeed-finalSpeed)/(oSpeed*DistanceUntilDecelerateInches);
  double leftSpeed = spd;
  double rightSpeed = spd;

   int i = 0;
   int j = 0;
  double distanceCoveredR =  0;
  double distanceCoveredL =  0;
  Point leftSide;
  Point rightSide;
  Point mid;
  leftSide.x = 0- EncoderDist/2;
  leftSide.y = 0;
  rightSide.x = 0 + EncoderDist/2;
  rightSide.y = 0;
  double prevDistanceL = 0;
  double prevDistanceR = 0;
  double rightArcLength = 0;
  double leftArcLength = 0;

  while(fabs(error)>=abs(1)){
    Point newLeftSide;
    Point newRightSide;
    bool isAccel = false;
    bool isDecel = false;
    distanceCoveredL =  ((distanL.position(degrees)/360)*M_PI*EncoderWheelDiameterInches);
    distanceCoveredR =  ((distanR.position(degrees)/360)*M_PI*EncoderWheelDiameterInches);
    leftArcLength = distanceCoveredL-prevDistanceL;
    rightArcLength = distanceCoveredR-prevDistanceR;
    //if(distanceCoveredL!=prevDistanceL && distanceCoveredR!=prevDistanceR){
      calculateNewPos(leftSide,rightSide,leftArcLength,rightArcLength,&newLeftSide,&newRightSide,EncoderDist);
      wait(500,msec);
      leftSide = newLeftSide;
      rightSide = newRightSide; 
    //}
    
    mid.x = (leftSide.x + rightSide.x)/2;
    mid.y = (leftSide.y + rightSide.y)/2;
    printf("\n mid x : %f\n", mid.x);
    printf("mid y : %f\n", mid.y);
    //printf("%f\n", mid.x);
    //  printf("%f\n", mid.y);
     // printf("%f\n", leftArcLength);
    //  printf("%f\n\n", rightArcLength);
    fflush(stdout);
    double alpha = atan((rightSide.y-leftSide.y)/(rightSide.x-leftSide.x));
    double epsilon = atan(fabs(mid.x)/(dist-mid.y));
    prevDistanceL = distanceCoveredL;
    prevDistanceR = distanceCoveredR;
    if(fabs(dist)<NonMaxSpeedDist){
      if(fabs(distanceCoveredR) < fabs(dist)*(DistanceUntilAccelerate/(NonMaxSpeedDist))){
        isAccel = true;
      }else{
        isDecel = true;
      }
    }else{
      if(fabs(distanceCoveredR)<=DistanceUntilAccelerate){
        isAccel = true;
      }else if(fabs(error)<=DistanceUntilDecelerateInches){
        isDecel = true;
      }  
    }
    if(isAccel){
      spd = initialSpeed*(1 + fabs(distanceCoveredR) * kAccel);
    }else if(isDecel){
      double distanceDecelerated = DistanceUntilDecelerateInches-fabs(error);
      spd = oSpeed*(1-fabs(distanceDecelerated)*kDecel);
    }else{
      spd = oSpeed;
    }

    double deltaTheta = epsilon+alpha;
    double speedCorrection = (spd/DistanceForMaxError)*deltaTheta;
    leftSpeed  = spd - speedCorrection;
    rightSpeed = spd + speedCorrection;
    if(dist < 0){
      double temp = leftSpeed;
      leftSpeed = -rightSpeed;
      rightSpeed = -temp;
    }
    if(i==7){
      printf("%f\n", deltaTheta);
    }
    if(j<printerSize&&i%printerSamplingRate==0){
      printer[j][0] = mid.x;
      printer[j][1] = mid.y;
      printer[j][2] = leftArcLength; 
      printer[j][3] = rightArcLength;
            j++;
    }
    i ++;
    
    LeftBack.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    error = sqrt(pow(mid.x,2)+pow(mid.y-dist,2));
  }
  leftDrive.stop();
  rightDrive.stop();
  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);
  Brain.Screen.print(inert.orientation(yaw,degrees));
  Brain.Screen.newLine();
  //turn(ang,angSpeed);
  printf("%d\n", i);
  /*for(int l = 0; l<printerSize;l++){
    printf("%f\n", printer[l][0]);
    printf("%f\n", printer[l][1]);
    printf("%f\n", printer[l][2]);
    printf("%f\n\n", printer[l][3]);
    fflush(stdout);

  }
  printf("Encoder value: \n");
  for(int l = 0; l<printerSize;l++){
    printf("%f\n", printer[l][1]);
    fflush(stdout);
  }*/
  fflush(stdout);
  
  
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
  //Brain.Screen.print("calibrated");
  wait(2000,msec);

  move(10,80,0,20);
  //turn(-45,40);
  //wait(.5,sec);
  //turn(-90,40);
 // wait(.5,sec);
 // turn(-135,40);
  //turn(0,40);
  
  
  //wait(1,sec);
  //printf("%f\n", inert.orientation(yaw,degrees));
  /*turn(-135,10);
  wait(1,sec);
  turn(-90,10);
  wait(1,sec);
  turn(0,10);
  vex::thread([](){
    intakeL(fwd,60,100);
  }).detach();
  vex::thread([](){
    intakeR(fwd,60,100);
  }).detach();
  move(28.75,80,-133,40);
  wait(.2,sec);
  move(27.25,80,-133,2);
  roller(.5,100);
  move(-10.25,80,-.5,40);
  wait(.2,sec);
  move(45.05,80,-90,40);
  //turn(-90,22);
  move(3.04,80,-90,2);
  roller(.5,100);
  wait(.4,sec);
  roller(.7,100);
  move(-4.94,80,95,40);
  move(24.5,80,-15,40);
  vex::thread([](){
    roller(.35,100);
  }).detach();
  move(42,80,-52.5,40);
  move(24.5,80,-52.5,10);
  roller(.5,100);
  move(-5,80,124,40);
  move(55,80,3,40);
  vex::thread([](){
    roller(.25,100);
  }).detach();
  move(25,80,7,20);
  roller(.5,100);
  wait(.4,sec);
  roller(.7,100);
  */
  //move(50,80,0,10);

  /*move(-15,80,-0,40);
  move(47,80,-60,40);
  move(19,80,-60,10);
  roller(.75,100);*/
  




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