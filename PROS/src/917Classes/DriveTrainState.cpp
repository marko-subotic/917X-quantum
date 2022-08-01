#include "DriveTrainState.hpp"
#include "Utils.hpp"
//#include "Globals.hpp"
#include "LockGuard.hpp"
#define _USE_MATH_DEFINES
#include <cstdio>
#include <math.h>

//pros::Mutex mtx;

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
    //LockGuard lockGuard(&mtx);
    return Point(m_x,m_y);
};


double DriveTrainState::getTheta(){
    //LockGuard lockGuard(&mtx);
    return m_theta;
}

void DriveTrainState::step(double dLeftEnc, double dRightEnc, double dBottomEnc, double dTheta){
    double rawRight = dRightEnc;
    double rawLeft = dLeftEnc;
    dLeftEnc = dLeftEnc/360*encWheelSize*M_PI;
    dRightEnc = dRightEnc / 360 * encWheelSize * M_PI;
    dBottomEnc = dBottomEnc / 360 * horEncWheelSize * M_PI;
	double shiftY; 
    double shiftX;
    if (!facingForward) {
        double temp = dRightEnc;
        dRightEnc = -dLeftEnc;
        dLeftEnc = -temp;
        dBottomEnc *= -1;
    }
    if(fabs(rawRight-rawLeft)<minimumForRotation){
        dTheta = 0;
        shiftY = (dLeftEnc + dRightEnc) / 2;
		shiftX = dBottomEnc;
    } else{
        double centerRotateY;
        double centerRotateX;
        centerRotateY = dBottomEnc / -dTheta;
        //These 2 if statements are to set the calculation point to either the left or right encoder
        //because the encoder further from the center of rotation is more accurate
        if(fabs(dRightEnc)>=fabs(dLeftEnc)){
            centerRotateX = dRightEnc / -dTheta;
            shiftY = sin(-dTheta / 2) * (centerRotateX + distanceYs / 2) * 2;

        }else{
            centerRotateX = dLeftEnc / -dTheta;
            shiftY = sin(-dTheta / 2) * (centerRotateX - distanceYs / 2) * 2;

        }
        //printf("dtheta: %f \n", dTheta);
        shiftX = sin(-dTheta / 2) * (centerRotateY + distanceX) * 2;
    }
    //printf("%f, %f, %f, %f\n", dLeftEnc, dRightEnc, dBottomEnc, dTheta);

    double deltaTheta = (fabs(rawLeft) + fabs(rawRight)) / 2;
    velocity = deltaTheta / loopDelay * 1000 / 360 * encWheelSize / bigDiam * 60 / rpms * 100;
   // LockGuard lockGuard(&mtx);
    m_x += shiftX * cos(-m_theta) + shiftY * sin(-m_theta);
    m_y += shiftY * cos(-m_theta) - shiftX * sin(-m_theta);
    m_theta = Utils::thetaConverter(m_theta + dTheta);
};


void DriveTrainState::step(double dLeftEnc, double dRightEnc, double dBottomEnc) {
    double leftInch = dLeftEnc / 360 * encWheelSize * M_PI;
    double rightInch = dRightEnc / 360 * encWheelSize * M_PI;
    step(dLeftEnc, dRightEnc, dBottomEnc, deltaTheta(leftInch, rightInch));
}

void DriveTrainState::switchDir() {
    m_theta = Utils::thetaConverter(m_theta + M_PI);
    facingForward = !facingForward;

}

double DriveTrainState::getVelo() {
    return velocity;
}

Point DriveTrainState::getCOR() {
    return centerRotation;
}


std::vector<double> DriveTrainState::calcAbsTheta(double leftEncAbs, double rightEncAbs) {
    abs_theta = (rightEncAbs - leftEncAbs) / 360 * encWheelSize * M_PI / distanceYs;
    int counter = 0;
    double dif = abs_theta * distanceYs;
    std::vector<double> rtrn(4, 0);
    while (abs_theta > 2 * M_PI || abs_theta < 0) {
        counter++;
        if (abs_theta > 2 * M_PI) {
            abs_theta -= M_PI * 2;
        }
        else if (abs_theta < 0) {
            abs_theta += M_PI * 2;
        }
    }
    rtrn[0] = counter;
    rtrn[1] = dif;
    rtrn[2] = leftEncAbs;
    rtrn[3] = rightEncAbs;
    return rtrn;
}