#include "main.h"
#include "math.h"
#include "Globals.hpp"
using namespace pros;

//DriveTrainState state(25,25, 0);

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
    bident.set_brake_mode(MOTOR_BRAKE_BRAKE);
    lift.set_brake_mode(MOTOR_BRAKE_BRAKE);
    bool clampToggle = false;
    double liftLock = lift.get_raw_position(NULL);
    double forkLock = bident.get_raw_position(NULL);
    lift.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    bident.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);

    while (true) {
        bool R2 = cont.get_digital(E_CONTROLLER_DIGITAL_R2);
        bool R1 = cont.get_digital(E_CONTROLLER_DIGITAL_R1);
        bool L2 = cont.get_digital(E_CONTROLLER_DIGITAL_L2);
        bool L1 = cont.get_digital(E_CONTROLLER_DIGITAL_L1);
        bool x = cont.get_digital_new_press(DIGITAL_X);

        if (R2) {
            lift.move(127);
            liftLock = lift.get_raw_position(NULL);
        }
        else if (R1) {
            lift.move(-127);
            liftLock = lift.get_raw_position(NULL);
        }if (!R1 && !R2) {
            lift.move_absolute(liftLock, 10);
        }
        if (L2) {
            bident.move(127);
            forkLock = bident.get_raw_position(NULL);
        }
        else if (L1) {
            bident.move(-127);
            forkLock = bident.get_raw_position(NULL);
        }if (!L1 && !L2) {
            bident.move_absolute(forkLock, 10);
        }
        if (x) {
            clampToggle = !clampToggle;
            clamp.set_value(clampToggle);
        }
        pros::delay(20);
    }
    clamp.set_value(false);
}
void opcontrol() {
    std::string driveTaskName("Drive Task");
    std::string intakeTaskName("Misc Task");
    Task driveTask(tankDrive, &driveTaskName);
    Task intakeTask(miscFunctions, &intakeTaskName);

    

}


