#define _USE_MATH_DEFINES
#include "DriveTrainState.hpp"
#include "Utils.hpp"
#include "LockGuard.hpp"
#include "pros/rtos.hpp"
#include <math.h>

pros::Mutex mtx;

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



Point DriveTrainState::getPos(){
    LockGuard lockGuard(&mtx);
    return Point(m_x,m_y);
};


double DriveTrainState::getTheta(){
    LockGuard lockGuard(&mtx);
    return m_theta;
}

void DriveTrainState::step(double dLeftEnc, double dRightEnc, double dBottomEnc){
    double rawRight = dRightEnc;
    double rawLeft = dLeftEnc;
    dLeftEnc = dLeftEnc/360*encWheelSize*M_PI;
    dRightEnc = dRightEnc / 360 * encWheelSize * M_PI;
    dBottomEnc = dBottomEnc / 360 * encWheelSize * M_PI;
    double dTheta = deltaTheta(dLeftEnc,dRightEnc);
	double shiftY; 
    double shiftX;
    if(fabs(rawRight-rawLeft)<minimumForRotation){
        shiftY = (dLeftEnc + dRightEnc) / 2;
		shiftX = dBottomEnc;
    } else{
        double centerRotateY;
        double centerRotateRX = rightEnc.x-dRightEnc/dTheta;
        double centerRotateLX = leftEnc.x-dLeftEnc/dTheta;
        centerRotateY = dBottomEnc / dTheta;
        Point centerRotation;
        //These 2 if statements are to set the calculation point to either the left or right encoder
        //because the encoder further from the center of rotation is more accurate
        if(fabs(dRightEnc)>=fabs(dLeftEnc)){
            centerRotation = Point(centerRotateRX, centerRotateY);
        }else{
            centerRotation = Point(centerRotateLX, centerRotateY);
        }
        
        Point lastPoint = Utils::rotateAroundPoint(centerRotation, calcPoint, dTheta);
        shiftX = lastPoint.x - calcPoint.x;
        shiftY = lastPoint.y - calcPoint.y;
    }
    LockGuard lockGuard(&mtx);
    m_x += shiftX * cos(-m_theta) + shiftY * sin(-m_theta);
    m_y += shiftY * cos(-m_theta) - shiftX * sin(-m_theta);
    m_theta = Utils::thetaConverter(m_theta + dTheta);
};