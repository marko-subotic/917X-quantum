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

vex::motor LeftBack = vex::motor(vex::PORT16,true);
vex::motor RightBack = vex::motor(vex::PORT15);
vex::motor LeftFront = vex::motor(vex::PORT14,true);
vex::motor RightFront = vex::motor(vex::PORT13);
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
  while(inert.isCalibrating()){
    wait(1,msec);
  }
  Controller1.Screen.print("Calibrated");

  
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
  const double Threshold = 5;
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
const double EncoderWheelDiameterInches = 2.795;
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
const double kDeltaX = 1;

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
  int counter = 1;
  double threshold = 0.013;
  bool bThresh = false;
  double deltaDist = threshold+1;
  double addingDeltaD = deltaDist;
  while(fabs(distanceCovered)<=fabs(dist)){
    wait(10,msec);
    counter ++;
    if(counter%20==0){
      if(fabs(addingDeltaD)<threshold){
        //printf("Distance: %f\n", dist);
        fflush(stdout);
        bThresh=true;
      }
      addingDeltaD = 0;
    }
      
    bool isAccel = false;
    bool isDecel = false;
    prevL = distanceCoveredL;
    prevR = distanceCoveredR;
    distanceCoveredR =  ((distanR.position(degrees)/360)*M_PI*EncoderWheelDiameterInches);
    distanceCoveredL =  ((distanL.position(degrees)/360)*M_PI*EncoderWheelDiameterInches);
    distanceCovered = (distanceCoveredR+distanceCoveredL)/2;
    deltaDist = ((distanceCoveredL-prevL)+(distanceCoveredR-prevR))/2;
    addingDeltaD+=deltaDist;
    if(bThresh){
      //printf("Distance: %f\n", dist);
      //printf("Acceleration: %f\n", inert.acceleration(yaxis));
      break;
    }
    //bThresh = true;
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
  if(bThresh){
    printf("Crashed: %f\n", inert.acceleration(yaxis));

  }
  Brain.Screen.newLine();
  //turn(ang,angSpeed);
  /*printf("i of move function: %d\n", i);
  for(int l = 0; l<printerSize;l++){
    printf("%f\n", printer[l][0]);
    fflush(stdout);
  }
  //printf("Encoder value: \n");
  for(int l = 0; l<printerSize;l++){
    printf("%f\n", printer[l][1]);
    fflush(stdout);
  }*/
  fflush(stdout);
  
  
}

void intake(directionType dir, double velocity){
  if(velocity==0){
    IntakeLeft.stop();
    IntakeRight.stop();
  }
  IntakeLeft.spin(dir, -velocity, vex::velocityUnits::pct);
  IntakeRight.spin(dir, -velocity, vex::velocityUnits::pct);

}
/*void intakeR(directionType dir, double velocity){
  IntakeRight.spin(dir, -velocity, vex::velocityUnits::pct);
}*/
void roller(double time, double velocity){
  BottomRoller.spinFor(fwd, time, sec, velocity, vex::velocityUnits::pct);
}

void skills(){
  double bft = .5;
  //t1
  intake(fwd,100);
  wait(.3,sec);
  //intakeR(fwd,100);
  move(25.25,80,-133,40);
  turn(-133,40);
  wait(.2,sec);
  move(33.25,80,-133,2);
  move(-bft,80,0,0);
  turn(-133,40);
  roller(.4,100);
  intake(fwd,0);

  //intake(fwd,0);
  //roller(.3,50);
  vex::thread([](){
    wait(.5,sec);
    roller(.2,-50);
  }).detach();
  move(-10.25,80,-.5,40);

//t2
 
  intake(reverse, 80);
  turn(2,40);
  intake(fwd, 100);
  wait(.2,sec);
  move(49,80,-90,40);
  // vex::thread([](){
   // roller(.3,100);
  //}).detach();

  turn(-90,40);
  move(10.04,80,-90,2);
  move(-bft,80,0,0);

  turn(-90,40);
  roller(.4,100);
  intake(fwd,0);
  wait(.4,sec);
  roller(.4,100);
  move(-9.64,80,95,40);

//t3
  intake(reverse, 20);
  vex::thread([](){
    roller(.5,-100);
  }).detach();
  turn(-95,40);
  turn(95,40);
  intake(fwd, 100);
  move(22.5,80,-15,40);
  turn(-12,40);
  //vex::thread([](){
   // roller(.35,100);
  //}).detach();
  move(49,80,-52.5,40);
  vex::thread([](){
    roller(.3,50);
  }).detach();
  turn(-54,40);
  move(27.5,80,-52.5,10);
  move(-bft,80,0,0);

  //turn(-52.5,40);
  roller(.45,100);
  move(-7.5,80,124,40);

//t4
  vex::thread([](){
    roller(.20,-60);
  }).detach();  
  intake(reverse, 40);
  turn(124.5,40);
  intake(fwd, 100);
  //vex::thread([](){
  //  roller(.35,-100);
  //}).detach();
  move(59,80,3,40);
  turn(2,40);
  vex::thread([](){
    roller(.25,30);
  }).detach();
  move(35,80,3,20);
  move(-bft,80,0,0);

  //turn(3,40);
  roller(.4,100);
  intake(fwd,0);
  wait(.4,sec);
  roller(.5,100);

//t5
  move(-3.3,80,-90,2);
  intake(reverse,40);
  vex::thread([](){
    roller(.5,-100);
  }).detach();
  turn(-35,40);
  turn(95,40);
  intake(fwd,100);
  move(47,80, 90,40);
  vex::thread([](){
    roller(.25,100);
  }).detach();
  turn(47,40);
  move(17.25,80,47,40);
  move(-bft,80,0,0);
  turn(47,40);
  roller(.8,100);
  //move(-13.25,80,47,40);
  

  //intake(fwd,0);
  //roller(.3,50);
  wait(.5, sec);
  move(-10.25,80,-.5,40);
  intake(reverse, 50);
  vex::thread([](){
    wait(.5,sec);
    roller(2,-100);
  }).detach();
  
  //intake(fwd,0);

//t6
 
  intake(reverse, 80);
  turn(-4+180,40);
  intake(fwd, 100);
  wait(.2,sec);
  move(49,80,-90,40);
  // vex::thread([](){
   // roller(.3,100);
  //}).detach();

  turn(-90+180,40);
  move(10.04,80,-90,2);
  move(-bft,80,0,0);

  turn(-90+180,40);
  roller(.4,100);
  intake(fwd,0);
  wait(.4,sec);
  roller(.4,100);
  move(-9.64,80,95,40);

//t7
  intake(reverse, 20);
  vex::thread([](){
    roller(.5,-100);
  }).detach();
  turn(-95+180,40);
  turn(95-180,40);
  intake(fwd, 100);
  move(22.5,80,-15,40);
  turn(-12+180,40);
  //vex::thread([](){
   // roller(.35,100);
  //}).detach();
  move(49,80,-52.5,40);
  vex::thread([](){
    roller(.3,50);
  }).detach();
  turn(-54+180,40);
  move(27.5,80,-52.5,10);
  move(-bft,80,0,0);

  //turn(-52.5,40);
  roller(.45,100);
  move(-7.5,80,124,40);

//t8
  vex::thread([](){
    roller(.20,-60);
  }).detach();  
  intake(reverse, 40);
  turn(124.5-180,40);
  intake(fwd, 100);
  //vex::thread([](){
  //  roller(.35,-100);
  //}).detach();
  move(59,80,3,40);
  turn(2-180,40);
  vex::thread([](){
    roller(.25,30);
  }).detach();
  move(35,80,3,20);
  move(-bft,80,0,0);

  //turn(3,40);
  roller(.4,100);
  intake(fwd,0);
  wait(.4,sec);
  roller(.5,100);

//t5
 /* move(-3.3,80,-90,2);
  intake(reverse,40);
  vex::thread([](){
    roller(.5,-100);
  }).detach();
  turn(-35,40);
  turn(95,40);
  intake(fwd,100);
  move(47,80, 90,40);
  vex::thread([](){
    roller(.25,100);
  }).detach();
  turn(47,40);
  move(15.25,80,47,40);
  move(-bft,80,0,0);
  turn(47,40);
  roller(.5,100);
  move(-13.25,80,47,40);
  intake(reverse, 50);
  vex::thread([](){
    roller(2,-100);
  }).detach();*/
  
}



void compHomeRow(int startingAng){
  
  intake(fwd,100);
  //15 second 2 tower auton
  //roller(.4,-100);
  move(27.75,80,-133-startingAng,40);
  wait(.2,sec);
  move(27.25,80,-133-startingAng,2);
  roller(.35,100);
  move(-11.25,80,92-startingAng,40);
  wait(.2,sec);
  vex::thread([](){
    roller(.4,100);
  }).detach();
  move(45.05,80,-179-startingAng,20);
  //turn(-90,22);
  move(2,80,-179-startingAng,20);
  roller(.5,100);
}

void compMidTow(int startingAng){
  
  intake(fwd,100);
  wait(.25,sec);
  //15 second 2 tower auton
  //roller(.4,-100);
  turn(140-startingAng,40);
  wait(.2, sec);
  move(3.87,80,-80-startingAng,40);
  vex::thread([](){
    wait(.97,sec);
    intake(fwd,0);
    }).detach();
  roller(.99,100);
  
  move(-3.87,80,-80-startingAng,40);
  
  wait(.3,sec);
  
  turn(-89-startingAng,40);
  intake(reverse, 30);
  vex::thread([](){
    wait(.4,sec);
    roller(.4,-100);
  }).detach();
  move(31,80,0,0);
  wait(.2,sec);
  turn(130,40);
  intake(fwd,100);
  move(20.7,80,0,0);
  move(-.5,80,0,0);
  roller(.8,100);
  intake(reverse,100);
  move(-7,80,0,0);
  vex::thread([](){
    roller(.4,-100);
  }).detach();
  turn(38-startingAng,40);
  intake(fwd,100);
  move(54,80,0,0);
  move(-3,80,0,0);
  turn(45-startingAng,40);
  vex::thread([](){
    wait(.4,sec);
    roller(1,100);
  }).detach();
  move(14,80,0,0);
  move(-40,80,0,0);



  //wait(.2,sec);
  /*move(29,80,-80-startingAng,2);
  turn(-122-startingAng,50);
  wait(.25,sec);
  vex::thread([](){
  roller(1.5,100);
  }).detach();
  wait(1.5,sec);
  move(-11.25,80,40-startingAng,40);

  turn(36-startingAng,50);

  wait(.2,sec);
  move(50,100,0-startingAng,40);
  turn(0-startingAng,50);

  vex::thread([](){
    roller(.4,100);
  }).detach();
  move(-7,80,45-startingAng,40);
  intake(fwd,0);
  turn(42-startingAng,50); 

  move(8,80,45-startingAng,50);
  vex::thread([](){
  roller(1,100);
  }).detach();
  //turn(45,50);
  //turn(45-startingAng,40);
  //turn(-90,22);
  //move(2,80,-179-startingAng,20);
  */
}

void comp2Tow(int startingAng){
  
  intake(fwd,100);
  wait(.25,sec);
  //15 second 2 tower auton
  //roller(.4,-100);
  move(26.75,80,-80-startingAng,40);
  turn(-120-startingAng,40);
  //wait(.2,sec);
  move(100.5,80,-80-startingAng,2);
  //turn(-131-startingAng,50);
  move(-.5,80,0,0);
  vex::thread([](){
  roller(.6,100);
  wait(.5,sec);
  roller(.5,100);

  }).detach();
  wait(.7,sec);
  intake(fwd,0);
  
  //wait(.8,sec);
  wait(1.5,sec);
  intake(reverse,20);
  move(-11.25,80,40-startingAng,40);

}




/////////////////////////////////////////////////////
//                                                 //
//                                                 //
//                     FULL AUTON                  //
//                                                 //
//                                                 //
/////////////////////////////////////////////////////

void autonomous(void) {
  
  
  
  //inert.setRotation(45,degrees);
  //Brain.Screen.print("calibrated");
 // wait(2000,msec);
  //turn(0,40);
  //move(30,80,0,40);
  TopRoller.spin(fwd,100,pct);
  Brain.Screen.print("Pressed");
  //skills();
  compMidTow(90);
  
  
  
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

static int takein = -100;
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
      IntakeLeft.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      IntakeRight.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
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
      TopRoller.spin(vex::directionType::fwd, -40, vex::velocityUnits::pct);

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
