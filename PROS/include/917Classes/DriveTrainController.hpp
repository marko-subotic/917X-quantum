#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include "structDefs.hpp"
#include "DriveTrainState.hpp"


class DriveTrainController {
    private:
        static constexpr double minErrorDegrees = .03;
        static constexpr double linSpd = 7;
        static constexpr double AngleUntilDecelerate = 3 * M_PI / 180;
        static constexpr double oSpeed = 100;
    public:
        //calculates length between 2 points, use it to calculate the radius from center of rotation
        //to the point calculated
        static void turnToPoint(DriveTrainState * state, Point target);
};
