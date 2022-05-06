#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include "structDefs.hpp"
#include "DriveTrainState.hpp"


class DriveTrainController {
private:
    //index of arrays is state of the bot, index 0 is 1 mogo in back, index 1 is no mogo
    static constexpr double minErrorDegrees = .7*M_PI/180;
    static constexpr double kProp [2] = { 80, 80 };
    static constexpr double kInteg[2] = { 13, 11 };
    static constexpr double kDer[2] = { 600, 600};
    


    //drive to point constants
    static constexpr int decelPow = 5;
    static constexpr double MinErrorInches[3] = { 2, 12, 11 }; //front: 9 back: 12
    static constexpr double DistanceUntilDecelerateInches[2] = { 16, 17 };
    static constexpr double DistanceUntilAccelerate = 0.1;
    static constexpr double kDist[2] = { .35,1 };
    static constexpr double AngleForMaxError[2] = { 15 * M_PI / 180,   14 * M_PI / 180 };
    //if final is greater than 20 risk of not finishing straight heightens
    static constexpr double finalSpeedForward[3] = { 30, 20, 20 };
    //if init is great than 35 risk of not going straight heightens
    static constexpr double initialSpeedForward[3] = { 20, 20, 20 };
    static constexpr double finalSpeedBackward[3] = { 20, 20, 30 };
    static constexpr double initialSpeedBackward[3] = { 20, 20, 90};
    static constexpr int loopDelay = 20;

    
public:
    static Point pointAligner(Point state, Point target, double finalAng, int distState);
    static void turnToPoint(DriveTrainState* state, Point target, double liftPos, int mogoState);
    static void driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState, double finalAng, double tiltPercent, double liftPercent, int radState, bool isClamping);
    static void driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState, double finalAng, bool isClamping);
    static void driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState, double finalAng, double tiltPercent, bool isClamping);
    static void intakeTask(int* state);
};
