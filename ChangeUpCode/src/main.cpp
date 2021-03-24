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
// distanR              encoder       G, H            
// inert                inertial      5               
// distanL              encoder       E, F            
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// distan               encoder       G, H            
// inert                inertial      5               
// distanL              encoder       E, F            
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

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;

// A global instance of competition
competition Competition;
int X2 = 0;
int Y1 = 0;
int X1 = 0;

// Declares all motors and the controller

vex::controller Controller1 = vex::controller();
vex::controller Controller2 = vex::controller();
//vex::controller Controller2 = vex::controller();

vex::motor LeftBack = vex::motor(vex::PORT19);
vex::motor RightBack = vex::motor(vex::PORT20, true);
vex::motor LeftFront = vex::motor(vex::PORT17);
vex::motor RightFront = vex::motor(vex::PORT18,true);
motor_group   leftDrive( LeftBack, LeftFront);
motor_group   rightDrive( RightBack, RightFront);
//vex::encoder dist = vex::encoder(vex::PORTA,vex::PORTB)

vex::motor IntakeLeft = vex::motor(vex::PORT3,true);
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
void pre_auton(void) { 
  vexcodeInit(); 
  inert.calibrate();

  
  }

///////////////////////////////////////////////////////////////
//                                                           //
//                                                           //
//                AUTON METHODS                              //
//                                                           //
//                                                           //
///////////////////////////////////////////////////////////////

//move (in tiles; + forward, - backwards)


bool hasCrashed(){
  const double Threshold = 1.5;
  if(fabs(inert.acceleration(yaxis))>Threshold){
    return true;
  }
  return false;
}
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
static const int printerSize = 100;
static const int numberIterations = 274922;
static double printer[printerSize][2];
static int printerSamplingRate = numberIterations/printerSize;


void turn(double ang, double spd)
{
  printf("Start: %f\n", inert.yaw(degrees));

  Brain.Screen.print(inert.orientation(yaw,degrees));
  Brain.Screen.newLine();
  double oSpeed = spd;
  double finalSpeed = 2;
  double initialSpeed = fmin(15,spd);
  spd = initialSpeed;
  double firstAngle = inert.yaw(degrees);
  double currentAngle = inert.yaw(degrees);
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

  double distanceCovered =  fabs(inert.yaw(degrees)-firstAngle);
  int i = 0;
  int j = 0;
  bool slept = false;
  while(fabs(distanceCovered)<=angNeeded - MinErrorDegrees){
    
    if(switcher>180||switcher<-180){
      if(firstAngle*inert.yaw(degrees)<0){
        if(inert.yaw(degrees)>0){
          distanceCovered = fabs(inert.yaw(degrees)-360-firstAngle);
        }else{
          distanceCovered = fabs(inert.yaw(degrees)+360-firstAngle);

        }
      }else{
        distanceCovered = fabs(inert.yaw(degrees)-firstAngle);

      }
    }else{
      distanceCovered = fabs(inert.yaw(degrees)-firstAngle);
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
      //printer[j][0] = distanceCovered;
      //printer[j][1] = spd;
      //      j++;
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
  //printf("%d\n", i);
  /*for(int l = 0; l<printerSize;l++){
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
  }*/
  //printf("%f\n", inert.orientation(yaw,degrees));
  //wait(1,sec);
  //printf("%f\n", inert.orientation(yaw,degrees));
  
}

void move(double dist, double inSpd,double ang, int angSpeed)
{
    double volatile spd = inSpd;
const double EncoderWheelDiameterInches = 4.33;
double DistanceUntilDecelerateInches = 20;
double DistanceUntilAccelerate = 7;
const double DistanceUntilLinearInches = 4;
const double AngleForMaxError = 90;
//if final is greater than 20 risk of not finishing straight heighens
const double finalSpeedForward = 30;
//if init is great than 35 risk of not going straight heightens
const double initialSpeedForward = 20;
const double finalSpeedBackward = 24;
const double initialSpeedBackward = 18;
const double kDeltaX = 25;

double finalSpeed = finalSpeedForward;
double initialSpeed = initialSpeedForward;

if(dist<0){
  double temp = DistanceUntilDecelerateInches;
  //DistanceUntilDecelerateInches = DistanceUntilAccelerate;
  //DistanceUntilAccelerate = temp;
  finalSpeed = finalSpeedBackward;
  //printf("final %f\n", finalSpeed);
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
  double firstAngle = inert.orientation(yaw,degrees);
  double currentAngle = inert.orientation(yaw,degrees);
  double error = dist;
  double kAccel = (oSpeed/(initialSpeed*DistanceUntilAccelerate))-(1/DistanceUntilAccelerate);
  double kDecel = (oSpeed-finalSpeed)/(oSpeed*DistanceUntilDecelerateInches);
  double leftSpeed = spd;
  double rightSpeed = spd;
  double prevAngle = currentAngle;
  double prevR = 0;
  double prevL = 0;
  double deltaX = 0;


   int i = 0;
   int j = 0;
  double distanceCovered =  0;
  double distanceCoveredR=  0;
  double distanceCoveredL =  0;

  while(fabs(distanceCovered)<=fabs(dist)){
    wait(10,msec);
    if(hasCrashed()){
      printf("Distance: %f\n", dist);
      printf("Acceleration: %f\n", inert.acceleration(yaxis));
      break;
    }
    bool isAccel = false;
    bool isDecel = false;
    prevL = distanceCoveredL;
    prevR = distanceCoveredR;
    distanceCoveredR =  ((distanR.position(degrees)/360)*M_PI*EncoderWheelDiameterInches);
    distanceCoveredL =  ((distanL.position(degrees)/360)*M_PI*EncoderWheelDiameterInches);
    distanceCovered = (distanceCoveredR+distanceCoveredL)/2;
    double deltaDist = ((distanceCoveredL-prevL)+(distanceCoveredR-prevR))/2;

    if(fabs(dist)<NonMaxSpeedDist){
      if(fabs(distanceCovered) < fabs(dist)*(DistanceUntilAccelerate/(NonMaxSpeedDist))){
        isAccel = true;
      }else{
        isDecel = true;
      }
    }else{
      if(fabs(distanceCovered)<=DistanceUntilAccelerate){
        isAccel = true;
      }else if(fabs(error)<=DistanceUntilDecelerateInches){
        isDecel = true;
      }  
    }
    if(isAccel){
      spd = initialSpeed*(1 + fabs(distanceCovered) * kAccel);
    }else if(isDecel){
      double distanceDecelerated = DistanceUntilDecelerateInches-fabs(error);
      spd = oSpeed*(1-fabs(distanceDecelerated)*kDecel);
    }else{
      spd = oSpeed;
    }
    prevAngle = currentAngle;
    currentAngle = inert.yaw(degrees);
    double averageAng = (prevAngle + currentAngle)/2;
    double alpha = averageAng-firstAngle;
    //alpha *=-1;
    double deltaTheta = currentAngle-firstAngle;
    deltaX += sin(alpha*M_PI/180)*deltaDist;
    double exp = 3;
    double speedCorrection = spd/oSpeed*((pow(deltaX,exp)*kDeltaX)+ (spd/AngleForMaxError)*deltaTheta);
    if(deltaX>1){
      speedCorrection = (pow(deltaX,1/exp)*kDeltaX);
    }
    leftSpeed  = spd- speedCorrection;//spd - speedCorrection;
    rightSpeed = spd+ speedCorrection;//spd + speedCorrection;
    if(dist < 0){
      double temp = leftSpeed;
      leftSpeed = -rightSpeed;
      rightSpeed = -temp;
    }
    
    if(j<printerSize&&i%printerSamplingRate==0){
      printer[j][0] = deltaX;
      printer[j][1] = inert.acceleration(yaxis);
            j++;
    }
    i ++;
    
    LeftBack.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd,leftSpeed,vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd,rightSpeed,vex::velocityUnits::pct);
    error = dist - (distanceCovered);
  }
  leftDrive.stop();
  rightDrive.stop();
  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);
  Brain.Screen.print(inert.orientation(yaw,degrees));
  Brain.Screen.newLine();
  turn(ang,angSpeed);
  printf("i of move function: %d\n", i);
  for(int l = 0; l<printerSize;l++){
    printf("%f\n", printer[l][0]);
    fflush(stdout);
  }/*
  //printf("Encoder value: \n");
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
  BottomRoller.spinFor(fwd, time, sec, velocity, vex::velocityUnits::pct);
}

void skills(){
  vex::thread([](){
    intakeL(fwd,60,100);
  }).detach();
  vex::thread([](){
    intakeR(fwd,60,100);
  }).detach();
  move(28.75,80,-133,40);
  wait(.2,sec);
  move(29.25,80,-133,2);
  roller(.5,100);
  move(-13.25,80,-.5,40);
  wait(.2,sec);
  move(45.05,80,-90,40);
  //turn(-90,22);
  move(4.04,80,-90,2);
  roller(.5,100);
  wait(.4,sec);
  roller(.7,100);
  move(-5.94,80,95,40);
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
}

void comp(){
  vex::thread([](){
    intakeL(fwd,60,100);
  }).detach();
  vex::thread([](){
    intakeR(fwd,60,100);
  }).detach();
  //15 second 2 tower auton
  //roller(.4,-100);
  move(27.75,80,-133,40);
  wait(.2,sec);
  move(27.25,80,-133,2);
  roller(.35,100);
  move(-11.25,80,92,40);
  wait(.2,sec);
  vex::thread([](){
    roller(.4,100);
  }).detach();
  move(45.05,80,-179,20);
  //turn(-90,22);
  move(2,80,-179,20);
  roller(.5,100);
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
  while(inert.isCalibrating()){
    wait(1,msec);
  }
  //inert.setRotation(45,degrees);
  //Brain.Screen.print("calibrated");
 // wait(2000,msec);
  //turn(0,40);
  move(30,80,0,40);
  Brain.Screen.print("Pressed");
  //skills();
  //comp();
  
  
  
  //wait(1,sec);
  //printf("%f\n", inert.orientation(yaw,degrees));
  
  
  
  
  
 
  




  

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
static int topRoller = 100;

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
  //Controller1.ButtonA.pressed(autonomous);

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

    //BottomRoller.spin(vex::directionType::fwd, -bottomRoller, vex::velocityUnits::pct);
    TopRoller.spin(vex::directionType::fwd, topRoller, vex::velocityUnits::pct);
    if(Controller1.ButtonL1.pressing()){
      BottomRoller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    } else if(Controller1.ButtonL2.pressing()){
      BottomRoller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
    } else{
      BottomRoller.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
    }

    int LeftSide1 = Controller1.Axis3.value();
    int RightSide1 = Controller1.Axis2.value();

    int LeftSide2 = Controller2.Axis3.value();
    int RightSide2 = Controller2.Axis2.value();

    int LeftSide;
    int RightSide;

    LeftSide = (LeftSide1 * LeftSide1 * LeftSide1) / (10000);

    if ((abs(LeftSide1) <= 10) and (abs(LeftSide2) >= 10)) {
      LeftSide = (LeftSide2 * LeftSide2 * LeftSide2) / (10000);
    }

    RightSide = (RightSide1 * RightSide1 * RightSide1) / (10000);

    if ((abs(RightSide1) <= 10) and (abs(RightSide2) >= 10)) {
      RightSide = (RightSide2 * RightSide2 * RightSide2) / (10000);
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
