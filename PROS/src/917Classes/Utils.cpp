#define _USE_MATH_DEFINES
#include "917Classes\Utils.hpp"
#include "math.h"

double Utils::thetaConverter(double theta) {
    if (theta < 0) {
        return theta + 2 * M_PI;
    }else if (theta > 2 * M_PI) {
        return theta - 2 * M_PI;
    }
    return theta;
};

Point Utils::rotateAroundPoint(Point pointOfRotation, Point pointRotating, double theta) {
    double cosT = cos(theta);
    double sinT = sin(theta);
    pointRotating.x -= pointOfRotation.x;
    pointRotating.y -= pointOfRotation.y;
    double xTemp = pointRotating.x;
    double yTemp = pointRotating.y;
    pointRotating.x = (xTemp * cosT) - (yTemp * sinT);
    pointRotating.y = (xTemp * sinT) + (yTemp * cosT);
    pointRotating.x += pointOfRotation.x;
    pointRotating.y += pointOfRotation.y;
    return pointRotating;
};