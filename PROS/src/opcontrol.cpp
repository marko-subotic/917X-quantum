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

void tankDrive(void* p) {
    int leftY = cont.get_analog(ANALOG_LEFT_Y);
    int rightY = cont.get_analog(ANALOG_RIGHT_Y);
    
    leftFront.move(0);
    leftBack.move(0);
    rightFront.move(0);
    rightBack.move(0);
    
    while (true) {
        leftY = cont.get_analog(ANALOG_LEFT_Y);
        rightY = cont.get_analog(ANALOG_RIGHT_Y);
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

void miscFunctions(void* p) {
    intake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    frontLift.set_brake_mode(MOTOR_BRAKE_BRAKE);
    while (true) {
        bool R2 = cont.get_digital(E_CONTROLLER_DIGITAL_R2);
        bool R1 = cont.get_digital(E_CONTROLLER_DIGITAL_R1);
        bool L2 = cont.get_digital(E_CONTROLLER_DIGITAL_L2);
        bool L1 = cont.get_digital(E_CONTROLLER_DIGITAL_L1);
        if (R2) {
            intake.move(127);
        }else if (R1) {
            intake.move(-127);
        }if(!R1 && !R2) {
            intake.move(0);
        }
        if (L2) {
            frontLift.move(127);
        }else if (L1) {
            frontLift.move(-127);
        }if (!L1 && !L2) {
            frontLift.move(0);
        }
        pros::delay(20);
    }
}
void opcontrol() {
    std::string driveTaskName("Drive Task");
    std::string intakeTaskName("Misc Task");
    Task driveTask(tankDrive, &driveTaskName);
    Task intakeTask(miscFunctions, &intakeTaskName);

    

}
