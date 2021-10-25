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


double DriveTrainState::thetaConverter(double thetaDeg){
    double rtrn = -thetaDeg*(M_PI/180);
    if(rtrn<0){
        rtrn+=2*M_PI;
    }
    return rtrn;
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


void DriveTrainState::step(double dLeftEnc, double dRightEnc, double dBottomEnc, double dTheta, double gyroReading){
    dLeftEnc = dLeftEnc/360*encWheelSize*M_PI;
    dRightEnc = dRightEnc / 360 * encWheelSize * M_PI;
    dBottomEnc = dBottomEnc / 360 * encWheelSize * M_PI;
    m_theta = thetaConverter(gyroReading);
    dTheta = -dTheta*(M_PI/180);
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
        /*
        double radius = distance2Points(centerRotation,calcPoint);
        double alpha = acos(encDist/(dTheta*radius));
        double mu = (M_PI-fabs(dTheta))/2;
        double gamma = M_PI-(alpha+mu);
        double rightEncoderLineDist = 2*radius*sin(fabs(dTheta)/2);

        //handling different quadrants of center of rotation to ensure shift in x and y
        //goes the right way.
        if(centerRotation.x>calcPoint.x){
            if(centerRotation.y>calcPoint.y){
                gamma -= M_PI_2;
            }else{
                gamma += M_PI;
            }
        } else{
            if(centerRotation.y<calcPoint.y){
                gamma += M_PI_2;
            }
        }
        shiftX = coefficient*cos(gamma)*rightEncoderLineDist;
        shiftY = coefficient*sin(gamma)*rightEncoderLineDist;
        //adding on shifts on x and y to centralize the point between 2 y encoders
        shiftX += -distanceYs/2*cos(fabs(dTheta));
        if(dTheta>0){
            shiftY -= distanceYs/2*sin(fabs(dTheta));
        }else{
            shiftY += distanceYs/2*sin(fabs(dTheta));
        }
        */
        Point lastPoint = rotateAroundPoint(centerRotation, calcPoint, dTheta);
        shiftX = lastPoint.x - calcPoint.x;
        shiftY = lastPoint.y - calcPoint.y;
    }
    m_y += shiftX*sin(m_theta) + shiftY * cos(m_theta);
    m_x += shiftX*cos(m_theta) - shiftY * sin(m_theta);
};