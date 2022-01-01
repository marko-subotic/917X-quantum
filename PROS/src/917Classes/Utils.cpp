#define _USE_MATH_DEFINES
#include "917Classes\Utils.hpp"
#include "917Classes\structDefs.hpp"
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

double Utils::angleToPoint(Point target) {
    if (target.y == 0) {
        if (target.x == 0) {
            return NULL;
        }else if (target.x>0) {
            return M_PI * 3 / 2;
        }else if (target.x < 0) {
            return M_PI / 2;
        }
    }
    double startAngle = fabs(atan(target.x / target.y));
    if (target.y > 0) {
        if (target.x <= 0) {
            return startAngle;
        }else {
            return 2 * M_PI - startAngle;
        }
    }else {
        if (target.x >= 0){
            return M_PI + startAngle;
        }
        else {
            return M_PI - startAngle;
        }
    }
    return 0;
};

double Utils::distanceBetweenPoints(Point one, Point two) {
    return sqrt(pow(one.x - two.x, 2) + pow(one.y - two.y, 2));
};