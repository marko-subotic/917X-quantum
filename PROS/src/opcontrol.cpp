#include "main.h"
using namespace pros;

int ScaleRawJoystick(int raw)
{
    // formula from https://www.vexforum.com/t/what-do-you-think-is-a-more-efficient-way-to-drive-your-robot/64857/42
    int ScaledVal = (pow(raw, JoystickScaleConst)) / (pow(127, JoystickScaleConst - 1));
    if ((JoystickScaleConst % 2 == 0) && (raw < 0))
        ScaledVal *= -1;

    return(ScaledVal);
}

void tankDrive() {
    int leftY = cont.get_analog(ANALOG_LEFT_Y);
    int rightY = cont.get_analog(ANALOG_RIGHT_Y);
    
    leftFront.move(0);
    leftBack.move(0);
    rightFront.move(0);
    rightBack.move(0);
    
    while (true) {

        if (abs(leftY) > DriveDeadzone) {
            leftY = ScaleRawJoystick(leftY);
        }
        if (abs(rightY) > DriveDeadzone) {
            rightY = ScaleRawJoystick(rightY);
        }
        leftFront.move(leftY);
        leftBack.move(leftY);
        rightFront.move(rightY);
        rightBack.move(rightY);
        pros::delay(20);
    }
}
void opcontrol() {

    
    

}
