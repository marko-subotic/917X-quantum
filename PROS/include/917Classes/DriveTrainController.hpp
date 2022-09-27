#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
//#include "main.h"
#include "constants.h"
#include "structDefs.hpp"
#include "DriveTrainState.hpp"


class DriveTrainController {
private:
    //index of arrays is state of the bot, index 0 is 1 mogo in back, index 1 is no mogo
    static constexpr double minErrorDegrees = 1.4*M_PI/180;
    static constexpr double kProp [2] = { 80, 80 };
    static constexpr double kInteg[2] = { 11, 11 };
    static constexpr double kDer[2] = { 600, 600};
    


    //drive to point constants
    static constexpr int decelPow = 5;
    static constexpr double MinErrorInches[4] = { 2, 12, 11, 15 }; //front: 9 back: 12
    static constexpr double DistanceUntilDecelerateInches[2] = { 16, 17 };
    static constexpr double DistanceUntilAccelerate = 0.1;
    static constexpr double AngleForMaxError[2] = { 15 * M_PI / 180,   14 * M_PI / 180 };
    //if final is greater than 20 risk of not finishing straight heightens
    static constexpr double finalSpeedForward[3] = { 30, 20, 20 };
    //if init is great than 35 risk of not going straight heightens
    static constexpr double initialSpeedForward[3] = { 20, 20, 20 };
    static constexpr double finalSpeedBackward[3] = { 20, 20, 30 };
    static constexpr double initialSpeedBackward[3] = { 20, 20, 90};
    static constexpr int loopDelay = 20;


    static constexpr int mogoWidth = 260;
    static constexpr double kMogo = .2;
    
    
    DriveTrainState* state;
    resetStruct* resetInterface;
public:

    //Constructor, pass in state and resetStruct pointers to be used as reference points for other functions
    DriveTrainController(DriveTrainState* state, resetStruct* resetInterface);


    /**
    * 
    *   Turns to point from current position,
    *        \param liftPos 
    *            Desired angle of lift
    *        \param mogoState  
    *           state of mogos on robot
    */
       
    void turnToPoint(Point target, double liftPos, int mogoState);

    /**
    *
    *   Drives to point from current position
    *        \param inSpd
    *           speed of movement
    *        \param liftPos
    *            Desired angle of lift
    *        \param mogoState
    *           state of mogos on robot
    *        \param isClamping
    *           if robot will clamp, determines radius of tolerance
    *        \param finalAng
    *           what angle the robot will come to the point at, shaped in a customizable inverse curve
    *        \param tiltPercent 
    *           through what percentage of the distance to the point will the back piston be deactivated
    *        \param liftPercent
    *           through what percentage of the distancce to the point will the lift move from current
    *           position to desired position
    *        
    *        
    */
    void driveToPoint(Point target, double inSpd, double liftPos, int mogoState, bool isClamping = false, double finalAng = ANGLE_IRRELEVANT, double tiltPercent = 110, double liftPercent = -1);

    
    
    static void intakeTask(int* state);
    void driveToMogo(Point target, double inSpd, double liftPos,int mogoState, bool isClamping = false, double finalAng = ANGLE_IRRELEVANT, double tiltPercent = 110, double liftPercent = -1, int colorID = BLUE_ID);
   
};
