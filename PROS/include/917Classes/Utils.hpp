#pragma once
#include "structDefs.hpp"
#include <cstdio>

namespace QuantumTest
{
    class QuantumTests;
};


class Utils{

    protected:
        static constexpr double kDist[3] = { .35, 1.0, .5 };
        static constexpr double mogoRad = 12.0;
    public:


        //allows testing class to access protected 
        friend class QuantumTest::QuantumTests;

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

        static double inertToRad(double deg);

        static Point pointAligner(Point state, Point target, double finalAng, int distState);

        static Point mogoReset(Point mogoP, double theta);
};