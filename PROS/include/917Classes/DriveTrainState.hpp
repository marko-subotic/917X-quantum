#pragma once
#include "structDefs.hpp"
namespace QuantumTest
{
    class QuantumTests;
};

class DriveTrainState{
    protected:
        double m_x;
        double m_y;
        double m_theta;
        const double encWheelSize = 1;// 2.783;
        //ratio of when wheel diameter is 1, so that i can change wheel size without messing up constants
        const double yRatio = 4.613;
        const double distanceYs = yRatio*encWheelSize;
        const double distanceX = 7.9375;
        const Point leftEnc = Point(-distanceYs/2, distanceX);
        const Point rightEnc = Point(distanceYs/2,distanceX);
        const Point bottomEnc = Point(0,0);
        const double minimumForRotation = .1;
        //This keeps  theta in  [0,2pi] range.
        double thetaConverter(double thetaDeg);
        //calculates length between 2 points, use it to calculate the radius from center of rotation
        //to the point calculated
        double distance2Points(Point first, Point second);
        
        //sets the the x and y to the param point vals and theta to param theta, used to reset/special cases
        //for testing
        void setState(Point resetPoint, double theta);
        //static method to calculate change in theta given the change in the 2 vertical encoders
        double deltaTheta(double leftEnc, double rightEnc);

    public:
	    //default constructor gives point and theta values to 0;
	     DriveTrainState();

        //Creates an object at the set coordinates and sets the constants to pre determined values
        DriveTrainState(double x, double y, double theta);

        //rotate point B around point A for theta radians, return new point B
        static Point rotateAroundPoint(Point pointOfRotation, Point pointRotating, double theta);

        //This will update the position of the object to the internal 3 main values m_x, m_y, m_theta
        void step(double dLeftEnc, double dRightEnc, double dBottomEnc);

        //returns the point of the m_x and m_y coordinates, no need to return theta because theta is 
        //read directly off of the sensor
        Point getPos();

        //returns theta for testing and such
        double getTheta();

        //allows testing class to access protected 
	    friend class QuantumTest::QuantumTests;

        
};