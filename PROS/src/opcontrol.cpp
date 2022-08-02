#include "main.h"
#include "math.h"
#include "Globals.hpp"
#include <algorithm>    // std::max

using namespace pros;
DriveTrainState stateO(0, 0, 0);
int takein = 2;
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
    if (takein != 127) {
        takein = 127;

    }
    else if (takein == 127) {
        takein = 0;
    }
}

void intakeOutFunc() {
    if (takein != -127) {
        takein = -127;
    }
    else if (takein == -127) {
        takein = 0;
    }
}

void tankDrive(void* p) {
    int leftY;
    int rightY;
    bool up;
    int coef = -1;
    bool facingForward = false;
    bool brake = false;
    leftFront.move(0);
    leftBack.move(0);
    rightFront.move(0);
    rightBack.move(0);
    
    while (true) {
        leftY = cont.get_analog(ANALOG_LEFT_Y);
        rightY = cont.get_analog(ANALOG_RIGHT_Y);
        up = cont.get_digital_new_press(DIGITAL_UP);
        bool right = cont.get_digital_new_press(DIGITAL_RIGHT);
        
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
        if (right) {
            brake = !brake;
            leftFront.tare_position();
            leftMid.tare_position();
            leftBack.tare_position();
            rightFront.tare_position();
            rightBack.tare_position();
            rightMid.tare_position();
        }
        if (brake) {
            

            leftFront.move_absolute(0, 100);
            leftMid.move_absolute(0, 100);
            leftBack.move_absolute(0, 100);
            rightFront.move_absolute(0, 100);
            rightBack.move_absolute(0, 100);
            rightMid.move_absolute(0, 100);
        }
        else {
            leftFront.move(coef * leftY);
            leftMid.move(coef * leftY);
            leftBack.move(coef * leftY);
            rightFront.move(coef * rightY);
            rightMid.move(coef * rightY);
            rightBack.move(coef * rightY);
        }
        
        /**/
        pros::delay(20);
    }

}

void miscFunctions(void* p) {
    lift.set_brake_mode(MOTOR_BRAKE_BRAKE);
    bool clampToggle = false;
    bool tiltToggle = false;
    bool coverToggle = false;
    bool highToggle = true;
    tilter.set_value(tiltToggle);
    double liftLock = lift.get_raw_position(NULL);
    lift.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    double prevLeft = leftEnc.get_position();
    double prevRight = rightEnc.get_position();
    while (true) {
        bool R2 = cont.get_digital(E_CONTROLLER_DIGITAL_R2);
        bool R1 = cont.get_digital(E_CONTROLLER_DIGITAL_R1);
        bool L2 = cont.get_digital_new_press(E_CONTROLLER_DIGITAL_L2);
        bool L1 = cont.get_digital_new_press(E_CONTROLLER_DIGITAL_L1);
        bool x = cont.get_digital_new_press(DIGITAL_X);
        bool y = cont.get_digital_new_press(DIGITAL_Y);
        bool left = cont.get_digital_new_press(DIGITAL_LEFT);
        bool b = cont.get_digital_new_press(DIGITAL_B);

        double deltaTheta = (fabs(leftEnc.get_position() / 100 - prevLeft) + fabs(rightEnc.get_position() / 100 - prevRight)) / 2;
        //double encoderRPM = deltaTheta / (loopDelay / 1000) / 360;
        double aveRealVelo = deltaTheta / 20 * 1000 / 360 * 2.75 / 4 * 60 / 300 * 100;//(fabs(rightMid.get_actual_velocity()) + fabs(rightBack.get_actual_velocity()) + fabs(rightFront.get_actual_velocity()) + fabs(leftMid.get_actual_velocity()) + fabs(leftBack.get_actual_velocity()) + fabs(leftFront.get_actual_velocity())) / 6;
        prevLeft = leftEnc.get_position() / 100, prevRight = rightEnc.get_position() / 100;
        //printf("%f\n", aveRealVelo);
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
        
        if (b) {
            highToggle = !highToggle;
            highReleaseL.set_value(highToggle);
            highReleaseR.set_value(highToggle);

        }
        if (L2) {
            intakeInFunc();

        }
        else if (L1) {
            intakeOutFunc();
            printf("re%d\n", takein);

        }
        pros::delay(20);
        intake.move(takein);
        if (x) {
            clampToggle = !clampToggle;
            clamp.set_value(clampToggle);
        }if (y) {
            tiltToggle = !tiltToggle;
            tilter.set_value(tiltToggle);
        }if (left) {
            coverToggle = !coverToggle;
            cover.set_value(coverToggle);
        }
        pros::delay(20);
    }
    clamp.set_value(false);
}
void intakeTask(void* p){
    //DriveTrainController::intakeTask(&takein);       
}
void odomFunctionsOP(void* p) {
    rightEnc.reset_position();
    if (!rightEnc.get_reversed()) {
        rightEnc.reverse();

    }
    leftEnc.reset_position();
    horEnc.reset_position();
    if (!horEnc.get_reversed()) {
        horEnc.reverse();
    }
    lv_obj_clean(lv_scr_act());
    OdomDisplay display(lv_scr_act());

    double prevRight = 0;
    double prevLeft = 0;
    double prevMid = 0;
    double covRight = rightEnc.get_position() / 100.0;
    double covLeft = leftEnc.get_position() / 100.0;
    double covMid = horEnc.get_position()/100.0;
    double deltaRight = covRight - prevRight;
    double deltaLeft = covLeft - prevLeft;
    double deltaMid = covMid - prevMid;
    double theta = inert.get_heading();
    double prevTheta = theta;
    double kInert = 1080.0 / (1080 + 12);

    display.setState(stateO.getPos(), theta);
    Point pointTwo(0, 0);
    while (1) {
        pros::delay(20);
        bool aPress = cont.get_digital_new_press(DIGITAL_LEFT);
        if (aPress) {
            std::vector<double> rtn = stateO.calcAbsTheta(covLeft, covRight);

            printf("%f, %f, %f, %f \n", rtn[0], rtn[1], rtn[2], rtn[3]);
        }

        if (std::max(fabs(deltaRight), std::max(fabs(deltaLeft), fabs(deltaMid))) < DriveTrainState::minTicks) {
            covRight = rightEnc.get_position() / 100.0, covLeft = leftEnc.get_position() / 100.0, covMid = horEnc.get_position() / 100.0;
            deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
            //printf("charging\n");
            continue;
        }
        //double pointAng = Utils::angleToPoint(Point(pointTwo.x - state.getPos().x, pointTwo.y - state.getPos().y));
        //double targetAng = pointAng - state.getTheta();
        //printf("%f, %f\n", covRight, covLeft);
        
        //printf("not charging\n");
        prevTheta = theta;
        theta = -Utils::inertToRad(inert.get_yaw());
        double dTheta = (theta - prevTheta) * kInert;

        if (dTheta > 2 * M_PI) {
            dTheta -= 2 * M_PI;
        }
        else if (dTheta < -2* M_PI) {
            dTheta += 2 * M_PI;
        }
        stateO.step(deltaLeft, deltaRight, deltaMid, dTheta);

        printf("%f, %f, %f, %f\n", deltaLeft, deltaRight, deltaMid, dTheta);

        display.setState(stateO.getPos(), theta);
        display.encoderDebug(covRight, "angle to point: ");
        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.get_position() / 100.0, covLeft = leftEnc.get_position() / 100.0, covMid = horEnc.get_position()/100.0;
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
    }
}


void angleCalibrate(void* p) {

    double hi = 13.5;
    double lo = 14.5;
    int status = 0;
   // stateO.distanceYs = lo + (hi - lo) / 2;
    while (true) {
        pros::delay(20);

        bool pressed = cont.get_digital_new_press(DIGITAL_A);
        bool bressed = cont.get_digital_new_press(DIGITAL_RIGHT);
        //printf("%f, %f\n", stateO.centerRotation.y - stateO.distanceX, );

        if (bressed) {
            stateO.setState(Point(0, 0), 0);

        }
        if (pressed) {
            
            
            status++;
            if (stateO.getTheta() < 3*M_PI/2) {
                hi = lo + (hi - lo) / 2;
            }
            else{
                lo = lo + (hi - lo) / 2;
            }
            //stateO.distanceYs = lo + (hi - lo) / 2;
            //printf("%f\n", stateO.distanceYs);
        }
        double bruh = 30;
        double straight = 30;
        
            
    }
    //DriveTrainController::intakeTask(&takein);       
}
void opcontrol() {
    //clamp.set_value(true);
    inAuton = false;
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftMid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    tilter.set_value(true);
    std::string driveTaskName("Drive Task");
    std::string intakeTaskName("Misc Task");
    pros::Task odomTasks(odomFunctionsOP);
    pros::Task calibrationTasks(angleCalibrate);
    //Task driveTask(tankDrive, &driveTaskName);
    Task intakeTask(miscFunctions, &intakeTaskName);

    

}


