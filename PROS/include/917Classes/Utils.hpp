#pragma once
#include "structDefs.hpp"

class Utils{
    public:
        //This keeps  theta in  [0,2pi] range.
        static double thetaConverter(double thetaDeg);

        //rotate point B around point A for theta radians, return new point B
        static Point rotateAroundPoint(Point pointOfRotation, Point pointRotating, double theta);

        //returns angle from the positive y-axis of any point to the origin, from [0,2PI)
        static double angleToPoint(Point target);

        //returns angle from positive y-axis in degrees, angle is positive to the right, used for auton
        static double angleBetweenPoints(Point one, Point two);


        //returns distance between 2 points
        static double distanceBetweenPoints(Point one, Point two);

        //returns voltage conversion from percentage
        static double perToVol(double percentage);

        static double redMotConv(double angle);
};