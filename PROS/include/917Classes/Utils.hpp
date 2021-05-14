#pragma once

class Utils{
    public:
        //This generates the number of the angle in degrees and accounts for disconuities using the system
        //left of forward is 0 to -180 and right of forward is 0 to 180. if turning left, angle is negative,
        //if turning right, angle is positive
        double calcInertAngle(double targetAng);

};