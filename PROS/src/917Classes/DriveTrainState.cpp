#define _USE_MATH_DEFINES
#include "917Classes\DriveTrainState.hpp"
#include <math.h>

DriveTrainState::DriveTrainState() {
    m_x = 0;
    m_y = 0;
    m_theta = 0;
};


DriveTrainState::DriveTrainState(double x, double y, double theta){
    m_x = x;
    m_y = y;
    m_theta = theta;
};


void DriveTrainState::setState(Point resetPoint, double theta) {
    m_x = resetPoint.x;
    m_y = resetPoint.y;
    m_theta = theta;
};

double DriveTrainState::deltaTheta(double leftEnc, double rightEnc) {
    return (rightEnc - leftEnc) / distanceYs;

};

Point DriveTrainState::rotateAroundPoint(Point pointOfRotation, Point pointRotating, double theta) {
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


double DriveTrainState::thetaConverter(double theta){
    if(theta<0){
        return theta+2*M_PI;
    }else if (theta < 2*M_PI) {
        return theta - 2 * M_PI;
    }
    return theta;
};



double DriveTrainState::distance2Points(Point first, Point second){
    return sqrt(pow(first.x-second.x,2)+pow(first.y-second.y,2));
};


Point DriveTrainState::getPos(){
    return Point(m_x,m_y);
};


double DriveTrainState::getTheta(){
    return m_theta;
}

void DriveTrainState::step(double dLeftEnc, double dRightEnc, double dBottomEnc){
    dLeftEnc = dLeftEnc/360*encWheelSize*M_PI;
    dRightEnc = dRightEnc / 360 * encWheelSize * M_PI;
    dBottomEnc = dBottomEnc / 360 * encWheelSize * M_PI;
    double dTheta = deltaTheta(dLeftEnc,dRightEnc);
    m_theta = thetaConverter(m_theta + dTheta);
    double coefficient = 1;
    if (dTheta<0){
        coefficient*=-1;
    }
	double shiftY; 
    double shiftX;
    if(fabs(dLeftEnc-dRightEnc)<minimumForRotation){
        shiftY = (dLeftEnc + dRightEnc) / 2;
		shiftX = dBottomEnc;
    } else{
        double centerRotateY;
        double centerRotateRX = rightEnc.x-dRightEnc/dTheta;
        double centerRotateLX = leftEnc.x-dLeftEnc/dTheta;
        Point centerRotation;
        Point calcPoint((rightEnc.x + leftEnc.x) / 2, (rightEnc.y + leftEnc.y) / 2);
        double encDist =dRightEnc;
        //These 2 if statements are to set the calculation point to either the left or right encoder
        //because the encoder further from the center of rotation is more accurate
        if(fabs(dRightEnc)>=fabs(dLeftEnc)){
            //if the center of rotation is to the right, the default setting is for the bottom encoder to
            //have a positive direction of right, so there have to be adjustments to where the center of
            //rotation will be if it is to the left or to the right.
            centerRotateY = dBottomEnc/dTheta;
            centerRotation = Point(centerRotateRX, centerRotateY);
        }else{
            centerRotateY = -dBottomEnc/dTheta;
            centerRotation = Point(centerRotateLX, centerRotateY);

        }
        
        Point lastPoint = rotateAroundPoint(centerRotation, calcPoint, dTheta);
        shiftX = lastPoint.x - calcPoint.x;
        shiftY = lastPoint.y - calcPoint.y;
    }
    m_y += shiftX*sin(m_theta) + shiftY * cos(m_theta);
    m_x += shiftX*cos(m_theta) - shiftY * sin(m_theta);
};

