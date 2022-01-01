#pragma once
#include "structDefs.hpp"

class DriveTrainController {
    private:

    public:
        //calculates length between 2 points, use it to calculate the radius from center of rotation
        //to the point calculated
        static double distance2Points(Point first, Point second);
};
