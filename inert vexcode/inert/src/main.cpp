/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\marko                                            */
/*    Created:      Wed Jul 27 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "DriveTrainState.h"
#include "Globals.cpp"
#include "Utils.h"
#include <iostream>

using namespace vex;


// A global instance of competition
competition Competition;
vex::rotation horEnc(14);

vex::controller cont;
vex::rotation rightEnc(15), leftEnc(13);
vex::inertial inert(11);
bool inAuton = true;

double kInert = 1080/(1080+12);
// define your global instances of motors and other devices here

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
  inert.calibrate();
  while(inert.isCalibrating()){
    wait(20,msec);
  }
  cont.Screen.print("bruh");

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
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
  // User control code here, inside the loop
  rightEnc.resetPosition();
    rightEnc.setReversed(true);
    leftEnc.resetPosition();
    horEnc.resetPosition();
        horEnc.setReversed(true);
    
    DriveTrainState stateO(0,0,0);
    double prevRight = 0;
    double prevLeft = 0;
    double prevMid = 0;
    double covRight = rightEnc.position(vex::rotationUnits::deg) ;
    double covLeft = leftEnc.position(vex::rotationUnits::deg) ;
    double covMid = horEnc.position(vex::rotationUnits::deg);
    double deltaRight = covRight - prevRight;
    double deltaLeft = covLeft - prevLeft;
    double deltaMid = covMid - prevMid;
    //inert.setHeading(0, vex::rotationUnits::deg);
    double theta = inert.heading();
    double prevTheta = theta; 
    Point pointTwo(0, 0);
    bool aPress = cont.ButtonLeft.pressing();

    while (1) {
        
        

        if (std::max(fabs(deltaRight), std::max(fabs(deltaLeft), fabs(deltaMid))) < DriveTrainState::minTicks) {
          
            covRight = rightEnc.position(vex::rotationUnits::deg) , covLeft = leftEnc.position(vex::rotationUnits::deg) , covMid = horEnc.position(vex::rotationUnits::deg) ;
            deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
            //printf("charging\n");
            continue;
        }
        //double pointAng = Utils::angleToPoint(Point(pointTwo.x - state.pos().x, pointTwo.y - state.pos().y));
        //double targetAng = pointAng - state.getTheta();
        //printf("%f, %f\n", covRight, covLeft);
        
        //printf("not charging\n");
        theta = Utils::inertToRad(inert.heading());
        double dTheta = (theta - prevTheta)*kInert;
        prevTheta = theta; 

        if(dTheta>2*M_PI){
          dTheta -= 2 * M_PI;
        }else if(dTheta<0){
          dTheta += 2 * M_PI;
        }
        stateO.step(deltaLeft, deltaRight, deltaMid, dTheta);
        printf("%f\n", inert.yaw(rotationUnits::deg));
        //std::cout << "right: " << inert.yaw(rotationUnits::deg) << std::endl; 
        //if (!aPress&&cont.ButtonLeft.pressing()) {
            //printf("%f, %f, %f, %f \n", leftEnc.position(vex::rotationUnits::deg), rightEnc.position(vex::rotationUnits::deg), horEnc.position(vex::rotationUnits::deg), dTheta);
        //}
        aPress = cont.ButtonLeft.pressing();
        //theta = stateO.getTheta();

        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.position(vex::rotationUnits::deg) , covLeft = leftEnc.position(vex::rotationUnits::deg) , covMid = horEnc.position(vex::rotationUnits::deg);
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
        wait(20,msec);
    }
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
    wait(100, msec);
  }
}
