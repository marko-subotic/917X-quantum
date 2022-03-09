#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include "structDefs.hpp"
#include "DriveTrainState.hpp"


class DriveTrainController {
private:
    //index of arrays is state of the bot, index 0 is 1 mogo in back, index 1 is no mogo
    static constexpr int division = 3;
    static constexpr int turnPow = 4;
    static constexpr double minErrorDegrees = 1*M_PI/180;
    static constexpr double linSpd[division][2] = { {25, 25},
                                                  {25, 20},
                                                  {25, 25}};
    static constexpr double AngleWhenDecelerate[division][2] = { { 50 * M_PI / 180, 45 * M_PI / 180 }, 
                                                        { 75 * M_PI / 180, 50 * M_PI / 180 }, 
                                                        { 107* M_PI / 180, 95 * M_PI / 180 } };
    static constexpr double AngleUntilLinear[division][2] = { { 7 * M_PI / 180, 5 * M_PI / 180 }, 
                                                     { 7 * M_PI / 180, 5 * M_PI / 180 }, 
                                                     { 7 * M_PI / 180, 5 * M_PI / 180 } };
    static constexpr double oSpeed[2] = { 100, 100};
    static constexpr int rpms = 300;
    static constexpr double bigDiam = 4.1;
    static constexpr double smallDiam = 2.75;
    static constexpr int loopDelay = 20;

    //drive to point constants
    static constexpr int decelPow = 5;
    static constexpr double MinErrorInches = 1;
    static constexpr double DistanceUntilDecelerateInches[2] = { 16, 13 };
    static constexpr double DistanceUntilAccelerate = 0.1;
    static constexpr double kCor[2] = { 1.3, .35 };
    static constexpr double AngleForMaxError[2] = { 7 * M_PI / 180, 13 * M_PI / 180 };
    //if final is greater than 20 risk of not finishing straight heightens
    static constexpr double finalSpeedForward[2] = { 30, 25 };
    //if init is great than 35 risk of not going straight heightens
    static constexpr double initialSpeedForward[2] = { 50, 50 };
    static constexpr double finalSpeedBackward[2] = { 30, 20 };
    static constexpr double initialSpeedBackward[2] = { 50, 127 };
public:
    static void turnToPoint(DriveTrainState* state, Point target, double liftPos, int mogoState);
    static void driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState);
};
