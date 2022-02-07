#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include "structDefs.hpp"
#include "DriveTrainState.hpp"


class DriveTrainController {
private:
    static constexpr double minErrorDegrees = .03;
    static constexpr double linSpd = 7;
    static constexpr double AngleUntilDecelerate = 28 * M_PI / 180;
    static constexpr double AngleUntilLinear = 3 * M_PI / 180;
    static constexpr double oSpeed = 100;

    //drive to point constants
    static constexpr double MinErrorInches = 1;
    static constexpr double DistanceUntilDecelerateInches = 27;
    static constexpr double DistanceUntilAccelerate = 7;
    static constexpr double AngleForMaxError = 22.5*M_PI/180;
    //if final is greater than 20 risk of not finishing straight heightens
    static constexpr double finalSpeedForward = 20;
    //if init is great than 35 risk of not going straight heightens
    static constexpr double initialSpeedForward = 50;
    static constexpr double finalSpeedBackward = 20;
    static constexpr double initialSpeedBackward = 50;
public:
    static void turnToPoint(DriveTrainState* state, Point target, double forkPos, double liftPos);
    static void driveToPoint(DriveTrainState* state, Point target, double inSpd, double forkPos, double liftPos);
};
