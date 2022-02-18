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
        const double encWheelSize = 2.775;
        const double horEncWheelSize = 2.775;
        const double distanceYs = 14.54;
        const double distanceX = 4.4;
        const Point leftEnc = Point(-distanceYs/2, distanceX);
        const Point rightEnc = Point(distanceYs/2,distanceX);
        const Point bottomEnc = Point(0,0);
        const Point calcPoint = Point((rightEnc.x + leftEnc.x) / 2, (rightEnc.y + leftEnc.y) / 2);
        const double minimumForRotation = 5;
        bool facingForward = true;

                    
        //sets the the x and y to the param point vals and theta to param theta, used to reset/special cases
        //for testing
        void setState(Point resetPoint, double theta);

        //static method to calculate change in theta given the change in the 2 vertical encoders
        double deltaTheta(double leftEnc, double rightEnc);

    public:
        static const int minTicks = 15;
	    //default constructor gives point and theta values to 0;
	    DriveTrainState();

        //Creates an object at the set coordinates and sets the constants to pre determined values
        DriveTrainState(double x, double y, double theta);


        //This will update the position of the object to the internal 3 main values m_x, m_y, m_theta
        void step(double dLeftEnc, double dRightEnc, double dBottomEnc);

        //returns the point of the m_x and m_y coordinates, n
        Point getPos();

        //returns theta for testing and such
        double getTheta();

        //switches direction, so i can apply turning to both sides of the bot
        void switchDir();

        //allows testing class to access protected 
	    friend class QuantumTest::QuantumTests;

        
};