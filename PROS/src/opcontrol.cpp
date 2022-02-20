#include "main.h"
#include "math.h"
#include "Globals.hpp"
#include <algorithm>    // std::max

using namespace pros;
DriveTrainState place(25, 25, 0);
int takein = 0;
const double intakemax = 127;

int ScaleRawJoystick(int raw)
{
    // formula from https://www.vexforum.com/t/what-do-you-think-is-a-more-efficient-way-to-drive-your-robot/64857/42
    int ScaledVal = (pow(raw, JoystickScaleConst)) / (pow(127, JoystickScaleConst - 1));
    if ((JoystickScaleConst % 2 == 0) && (raw < 0))
        ScaledVal *= -1;

    return(ScaledVal);
}

void intakeInFunc() {
    if (takein != intakemax) {
        takein = intakemax;
    }
    else if (takein == intakemax) {
        takein = 0;
    }
}

void intakeOutFunc() {
    if (takein != -intakemax) {
        takein = -intakemax;
    }
    else if (takein == -intakemax) {
        takein = 0;
    }
}

void tankDrive(void* p) {
    int leftY;
    int rightY;
    bool up;
    int coef = -1;
    bool facingForward = false;
    leftFront.move(0);
    leftBack.move(0);
    rightFront.move(0);
    rightBack.move(0);
    while (true) {
        leftY = cont.get_analog(ANALOG_LEFT_Y);
        rightY = cont.get_analog(ANALOG_RIGHT_Y);
        bool up = cont.get_digital_new_press(DIGITAL_UP);


        if (abs(leftY) > DriveDeadzone) {
            leftY = ScaleRawJoystick(leftY);
        }
        if (abs(rightY) > DriveDeadzone) {
            rightY = ScaleRawJoystick(rightY);
        }
        if (up) {
            coef *= -1;
            facingForward = !facingForward;
        }if (!facingForward) {
            int sub = rightY;
            rightY = leftY;
            leftY = sub;
        }
        leftFront.move(coef * leftY);
        leftMid.move(coef * leftY);
        leftBack.move(coef * leftY);
        rightFront.move(coef * rightY);
        rightMid.move(coef * rightY);
        rightBack.move(coef * rightY);
        /**/
        pros::delay(20);
    }

}

void miscFunctions(void* p) {
    lift.set_brake_mode(MOTOR_BRAKE_BRAKE);
    bool clampToggle = false;
    bool tiltToggle = true;
    tilter.set_value(tiltToggle);
    double liftLock = lift.get_raw_position(NULL);
    lift.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);

    while (true) {
        bool R2 = cont.get_digital(E_CONTROLLER_DIGITAL_R2);
        bool R1 = cont.get_digital(E_CONTROLLER_DIGITAL_R1);
        bool L2 = cont.get_digital_new_press(E_CONTROLLER_DIGITAL_L2);
        bool L1 = cont.get_digital_new_press(E_CONTROLLER_DIGITAL_L1);
        bool x = cont.get_digital_new_press(DIGITAL_X);
        bool y = cont.get_digital_new_press(DIGITAL_Y);


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
            intakeInFunc();
        }
        else if (L1) {
            intakeOutFunc();
        }
        intake.move(takein);
        if (x) {
            clampToggle = !clampToggle;
            clamp.set_value(clampToggle);
        }if (y) {
            tiltToggle = !tiltToggle;
            tilter.set_value(tiltToggle);
        }
        pros::delay(20);
    }
    clamp.set_value(false);
}

void odomFunctionsOP(void* p) {
    rightEnc.reset();
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
    double theta = place.getTheta();
    display.setState(place.getPos(), theta);
    Point pointTwo(0, 0);
    while (1) {
        
        //double pointAng = Utils::angleToPoint(Point(pointTwo.x - place.getPos().x, pointTwo.y - place.getPos().y));
        //double targetAng = pointAng - place.getTheta();
        printf("%f\n", std::max(deltaRight, std::max(deltaLeft, deltaMid)));
        if (std::max(fabs(deltaRight), std::max(fabs(deltaLeft), fabs(deltaMid))) < DriveTrainState::minTicks) {
            covRight = rightEnc.get_value(), covLeft = leftEnc.get_value(), covMid = horEnc.get_value();
            deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
            //printf("charging\n");
            continue;
        }
        //printf("not charging\n");

        place.step(deltaLeft, deltaRight, deltaMid);
        theta = place.getTheta();
        display.setState(place.getPos(), theta);
        display.encoderDebug(covRight, "angle to point: ");
        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.get_value(), covLeft = leftEnc.get_value(), covMid = horEnc.get_value();
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
        pros::delay(1);
    }
}

void opcontrol() {
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftMid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    std::string driveTaskName("Drive Task");
    std::string intakeTaskName("Misc Task");
    pros::Task odomTasks(odomFunctionsOP);
    Task driveTask(tankDrive, &driveTaskName);
    Task intakeTask(miscFunctions, &intakeTaskName);

    

}


