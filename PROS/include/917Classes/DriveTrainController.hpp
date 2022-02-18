#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include "structDefs.hpp"
#include "DriveTrainState.hpp"


class DriveTrainController {
private:
    //index of arrays is state of the bot, index 0 is 1 mogo in back, index 1 is no mogo
    static constexpr double minErrorDegrees = .04;
    static constexpr double linSpd = 7;
    static constexpr double AngleUntilDecelerate [2] = { 0 * M_PI / 180, 0 * M_PI / 180 };
    static constexpr double AngleUntilLinear[2] = { 7 * M_PI / 180, 7 * M_PI / 180 };
    static constexpr double oSpeed[2] = { 80, 70};

    //drive to point constants
    static constexpr double MinErrorInches = 1;
    static constexpr double DistanceUntilDecelerateInches = 27;
    static constexpr double DistanceUntilAccelerate = 7;
    static constexpr double AngleForMaxError[2] = { 7.5 * M_PI / 180, 14 * M_PI / 180 };
    //if final is greater than 20 risk of not finishing straight heightens
    static constexpr double finalSpeedForward = 20;
    //if init is great than 35 risk of not going straight heightens
    static constexpr double initialSpeedForward = 50;
    static constexpr double finalSpeedBackward = 15;
    static constexpr double initialSpeedBackward = 50;
public:
    static void turnToPoint(DriveTrainState* state, Point target, double liftPos, int mogoState);
    static void driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState);
};
