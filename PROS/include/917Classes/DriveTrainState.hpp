#pragma once
#include "structDefs.hpp"

class DriveTrainState{
    private:
        double m_x;
        double m_y;
        double m_theta;
        const double encWheelSize = 2.783;
        const double distanceYs = 11.875;
        const Point leftEnc = Point(-distanceYs/2, distanceX);
        const Point rightEnc = Point(distanceYs/2,distanceX);
        const Point bottomEnc = Point(0,0);
        const double distanceX = 7.125;
        const double minimumForRotation = 0.30;
        //This converts the reading of the gyro from -180 to 180 into radians.
        double thetaConverter(double thetaDeg);
        //calculates length between 2 points, use it to calculate the radius from center of rotation
        //to the point calculated
        double distance2Points(Point first, Point second);
    public:

        //Creates an object at the set coordinates and sets the constants to pre determined values
        DriveTrainState(double x, double y, double theta);
        //This will update the position of the object to the internal 3 main values m_x, m_y, m_theta
        void step(double dLeftEnc, double dRightEnc, double dBottomEnc, double dTheta, double theta);
        //returns the point of the m_x and m_y coordinates, no need to return theta because theta is 
        //read directly off of the sensor
        Point getPos();

        double getTheta();
};