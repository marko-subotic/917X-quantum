#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include "structDefs.hpp"
#include "DriveTrainState.hpp"


class DriveTrainController {
private:
    //index of arrays is state of the bot, index 0 is 1 mogo in back, index 1 is no mogo
    static constexpr double minErrorDegrees = .03;
    static constexpr double linSpd = 3;
    static constexpr double AngleUntilDecelerate [2] = { 37 * M_PI / 180, 37  * M_PI / 180 };
    static constexpr double AngleUntilLinear[2] = { 8 * M_PI / 180, 8 * M_PI / 180 };
    static constexpr double oSpeed[2] = { 40, 40};

    //drive to point constants
    static constexpr int decelPow = 5;
    static constexpr double MinErrorInches = 1;
    static constexpr double DistanceUntilDecelerateInches[2] = { 16, 13 };
    static constexpr double DistanceUntilAccelerate = 4;
    static constexpr double AngleForMaxError[2] = { 7 * M_PI / 180, 13 * M_PI / 180 };
    //if final is greater than 20 risk of not finishing straight heightens
    static constexpr double finalSpeedForward[2] = { 30, 25 };
    //if init is great than 35 risk of not going straight heightens
    static constexpr double initialSpeedForward[2] = { 80, 80 };
    static constexpr double finalSpeedBackward[2] = { 30, 30 };
    static constexpr double initialSpeedBackward[2] = { 80, 80 };
public:
    static void turnToPoint(DriveTrainState* state, Point target, double liftPos, int mogoState);
    static void driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState);
};
