#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include "structDefs.hpp"
#include "DriveTrainState.hpp"


class DriveTrainController {
private:
    //index of arrays is state of the bot, index 0 is 1 mogo in back, index 1 is no mogo
    static constexpr double minErrorDegrees = .5*M_PI/180;
    static constexpr double kProp [2] = { 145, 95 };
    static constexpr double kInteg[2] = { 17, 15 };
    static constexpr double kDer[2] = { 505, 495};
    
    static constexpr int loopDelay = 20;


    //drive to point constants
    static constexpr int decelPow = 5;
    static constexpr double kDist = .35;
    static constexpr double MinErrorInches = 2;
    static constexpr double DistanceUntilDecelerateInches[2] = { 16, 13 };
    static constexpr double DistanceUntilAccelerate = 0.1;
    static constexpr double kCor[2] = { 1.3, .35 };
    static constexpr double AngleForMaxError[2] = { 17 * M_PI / 180,   20 * M_PI / 180 };
    //if final is greater than 20 risk of not finishing straight heightens
    static constexpr double finalSpeedForward[2] = { 30, 20 };
    //if init is great than 35 risk of not going straight heightens
    static constexpr double initialSpeedForward[2] = { 50, 50 };
    static constexpr double finalSpeedBackward[2] = { 30, 20 };
    static constexpr double initialSpeedBackward[2] = { 50, 127 };

    static constexpr int rpms = 300;
    static constexpr double bigDiam = 4.1;
    static constexpr double smallDiam = 2.75;
    
public:
    static Point pointAligner(Point state, Point target, double finalAng);
    static void turnToPoint(DriveTrainState* state, Point target, double liftPos, int mogoState);
    static void driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState, double finalAng, double tiltPercent, double liftPercent);
    static void driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState, double finalAng);
    static void driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState, double finalAng, double tiltPercent);

};
