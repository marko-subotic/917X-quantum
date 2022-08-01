#define _USE_MATH_DEFINES
#include "Utils.hpp"
#include "structDefs.hpp"
#include "math.h"
#include "constants.h"
#include <cassert>


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

    double xTemp = pointRotating.x - pointOfRotation.x;
    double yTemp = pointRotating.y - pointOfRotation.y;
    pointOfRotation.x += (xTemp * cosT) - (yTemp * sinT);
    pointOfRotation.y += (xTemp * sinT) + (yTemp * cosT);

    return pointOfRotation;
};

double Utils::angleToPoint(Point target) {
    if (target.y == 0) {
        if (target.x == 0) {
            //assert(false, "dividing 0/0");
            return 300;
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

double Utils::angleBetweenPoints(Point one, Point two) {
    return -180 * M_PI * Utils::angleToPoint(Point(one.x - two.x, one.y - two.y));
};

double Utils::distanceBetweenPoints(Point one, Point two) {
    return sqrt(pow(one.x - two.x, 2) + pow(one.y - two.y, 2));
};

double Utils::perToVol(double percentage) {
    return percentage * 127 / 100;

};

double Utils::redMotConv(double angle) {
    return angle / 360 * LIFT_ENCODER;
};

double Utils::inertToRad(double deg) {
    double converted = deg * M_PI / 180;
    if (converted < 0) {
        converted += 2 * M_PI;
    }
    else if (converted > M_PI * 2) {
        converted -= 2 * M_PI;
    }
    return converted;
};

Point Utils::pointAligner(Point state, Point target, double finalAng, int distState) {
    double xPerp, yPerp;
    Point alignPoint(0, 0);
    if (fabs(finalAng) < .01 || fabs(fabs(finalAng) - M_PI) < .01) {
        xPerp = target.x;
        yPerp = state.y;
    }
    else if (fabs(fabs(finalAng) - M_PI / 2.0) < .01 || fabs(fabs(finalAng) - 3 * M_PI / 2) < .01) {
        xPerp = state.x;
        yPerp = target.y;
    }
    else {
        xPerp = (state.y - target.y + 1 / tan(finalAng) * target.x + state.x * tan(finalAng)) / (1 / tan(finalAng) + tan(finalAng));
        yPerp = (-tan(finalAng)) * (xPerp - state.x) + state.y;
        //printf("%f, %f\n", xPerp, yPerp);

    }
    printf("distState: %d\n", distState);
    alignPoint.x = xPerp + (target.x - xPerp) * kDist[distState];
    alignPoint.y = yPerp + (target.y - yPerp) * kDist[distState];
    return alignPoint;
}

Point Utils::mogoReset(Point mogoP, double theta) {
    return Point(mogoP.x - sin(theta) * mogoRad, mogoP.y + cos(theta) * mogoRad);
}
