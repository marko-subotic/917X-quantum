#pragma once
#include "917Classes\structDefs.hpp"

class Utils{
    public:
        //This keeps  theta in  [0,2pi] range.
        static double thetaConverter(double thetaDeg);

        //rotate point B around point A for theta radians, return new point B
        static Point rotateAroundPoint(Point pointOfRotation, Point pointRotating, double theta);

        static double angleToPoint(Point target) {

        }
};