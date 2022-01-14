#include "main.h"
#include "math.h"
using namespace pros;

DriveTrainState state(25,25, 0);

int ScaleRawJoystick(int raw)
{
    // formula from https://www.vexforum.com/t/what-do-you-think-is-a-more-efficient-way-to-drive-your-robot/64857/42
    int ScaledVal = (pow(raw, JoystickScaleConst)) / (pow(127, JoystickScaleConst - 1));
    if ((JoystickScaleConst % 2 == 0) && (raw < 0))
        ScaledVal *= -1;

    return(ScaledVal);
}

void tankDrive(void* p) {
    int leftY;
    int rightY;
    bool up;
    int coef = 1;

    leftFront.move(0);
    leftBack.move(0);
    rightFront.move(0);
    rightBack.move(0);
    
    while (true) {
        leftY = cont.get_analog(ANALOG_LEFT_Y);
        rightY = cont.get_analog(ANALOG_RIGHT_Y);
        bool up = cont.get_digital_new_press(DIGITAL_UP);

        if (up) {
            coef *= -1;
        }
        if (abs(leftY) > DriveDeadzone) {
            leftY = ScaleRawJoystick(leftY);
        }
        if (abs(rightY) > DriveDeadzone) {
            rightY = ScaleRawJoystick(rightY);
        }
        leftFront.move(coef*leftY);
        leftMid.move(coef * leftY);
        leftBack.move(coef * leftY);
        rightFront.move(coef * rightY);
        rightMid.move(coef * rightY);
        rightBack.move(coef * rightY);
        pros::delay(20);
    }
}

void miscFunctions(void* p) {
    intake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    frontLift.set_brake_mode(MOTOR_BRAKE_BRAKE);
    bool clampToggle = false;
    while (true) {
        bool R2 = cont.get_digital(E_CONTROLLER_DIGITAL_R2);
        bool R1 = cont.get_digital(E_CONTROLLER_DIGITAL_R1);
        bool L2 = cont.get_digital(E_CONTROLLER_DIGITAL_L2);
        bool L1 = cont.get_digital(E_CONTROLLER_DIGITAL_L1);
        bool x = cont.get_digital_new_press(DIGITAL_X);

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

/*
void odomFunctions(void* p) {
    
    .reset();
    leftEnc.reset();
    horEnc.reset();
    lv_obj_clean(lv_scr_act());
    OdomDisplay display(lv_scr_act());
    double prevRight = 0;
    double prevLeft = 0;
    double prevMid = 0;
    double covRight = rightEnc.get_value();
    double covLeft = leftEnc.get_value();
    double covMid = horEnc.get_value();
    double deltaRight = covRight - prevRight;
    double deltaLeft = covLeft - prevLeft;
    double deltaMid = covMid - prevMid;
    double theta = 0;
    while (1) {
        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.get_value(), covLeft = leftEnc.get_value(), covMid = horEnc.get_value();
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
        state.step(deltaLeft, deltaRight, deltaMid);
        theta = state.getTheta();
        display.setState(state.getPos(), theta*180/M_PI);
        display.encoderDebug(covMid, "rightEncoder: ");
        pros::delay(20);
    }
}

void motorControl(void* p) {
    Point pointOne(0,0);
    DriveTrainController::turnToPoint(&state, pointOne);
}
void opcontrol() {
    pros::Task odomTasks(odomFunctions);


}
*/
