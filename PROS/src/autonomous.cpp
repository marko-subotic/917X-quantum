#include "main.h"
#include "917Classes/DriveTrainState.hpp"
#include <algorithm>    // std::max



//DriveTrainState state(110.5, 12, M_PI);
//DriveTrainState state(27, 10.5, M_PI/2);
DriveTrainState state(25, 25, 0);

void odomFunctions(void* p) {
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
    double theta = state.getTheta();
    display.setState(state.getPos(), theta);
    Point pointTwo(0, 0);
    while (1) {

        //double pointAng = Utils::angleToPoint(Point(pointTwo.x - state.getPos().x, pointTwo.y - state.getPos().y));
        //double targetAng = pointAng - state.getTheta();
        //printf("%f\n", std::max(deltaRight, std::max(deltaLeft, deltaMid)));
        if (std::max(fabs(deltaRight), std::max(fabs(deltaLeft), fabs(deltaMid))) < DriveTrainState::minTicks) {
            covRight = rightEnc.get_value(), covLeft = leftEnc.get_value(), covMid = horEnc.get_value();
            deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
            //printf("charging\n");
            continue;
        }
        //printf("not charging\n");

        state.step(deltaLeft, deltaRight, deltaMid);
        theta = state.getTheta();
        display.setState(state.getPos(), theta);
        display.encoderDebug(covMid, "angle to point: ");
        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.get_value(), covLeft = leftEnc.get_value(), covMid = horEnc.get_value();
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
        pros::delay(20);
    }
}

void rightSide(void* p) {
    Point pointOne(108, 58.5);
    Point pointTwo(108, 45);
    Point pointThree(115, 43);
    Point pointFour(125, 40);
    Point pointFive(122, 42);
    Point pointSix(100, 0);
    printf("looping through motor");
    //DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    DriveTrainController::driveToPoint(&state, pointOne, -100, 10,1);
    clamp.set_value(true);
    DriveTrainController::driveToPoint(&state, pointTwo, 100, 0,0);
    DriveTrainController::turnToPoint(&state, pointThree, Utils::redMotConv(-10), 0);
    DriveTrainController::driveToPoint(&state, pointThree,100, -10, 0);
    pros::delay(700);

    //DriveTrainController::turnToPoint(&state, pointTwo, .25, -10);
    //DriveTrainController::driveToPoint(&state, pointTwo, 100, .2, -10);
    DriveTrainController::driveToPoint(&state, Point(state.getPos().x - 10, fabs(atan(state.getTheta())) * 10 + state.getPos().y), -100, Utils::redMotConv(-10),0);
    DriveTrainController::turnToPoint(&state, pointFive, Utils::redMotConv(-10), 0);
    DriveTrainController::driveToPoint(&state, pointFive, 100, Utils::redMotConv(-10),0);
    pros::delay(2000);
    DriveTrainController::turnToPoint(&state, pointSix, Utils::redMotConv(-10),0);

}

void prog(void* p) {
    
    lift.set_zero_position(lift.get_raw_position(NULL));
    Point pointOne(37, 59);
    Point pointTwo(73, 107);
    Point pointThree(67, 96);
    Point pointFour(64, 91);
    Point pointFive(70, 99);
    Point pointSix(73, 105);
    Point pointSeven(24, 115);
    Point pointEight(64, 74);
    Point pointNine(64, 28);
    printf("%f\n", Utils::perToVol(100));
    
    tilter.set_value(true);
    DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    pros::delay(100);
    intake.move(-127);

    //pros::delay(1000);
    DriveTrainController::driveToPoint(&state, pointOne, -100, 0, 0);
    clamp.set_value(true);
    pros::delay(500);
    
    DriveTrainController::turnToPoint(&state, pointTwo, -100, 0);
    DriveTrainController::driveToPoint(&state, pointTwo, -100, -100, 0);
    lift.move_absolute(Utils::redMotConv(-79), 100);
    pros::delay(500);
    clamp.set_value(false);
    lift.move_absolute(Utils::redMotConv(-87), 100);
    pros::delay(500);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointThree, -100, 0);
    DriveTrainController::driveToPoint(&state, pointThree, 100, -87, 0);
    tilter.set_value(false);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointFive, 0, 1);
    DriveTrainController::driveToPoint(&state, pointFive, -100, 0, 1);
    DriveTrainController::turnToPoint(&state, pointFour, -20, 1);
    DriveTrainController::driveToPoint(&state, pointFour, -100, 4, 1);
    clamp.set_value(true);
    DriveTrainController::turnToPoint(&state, pointSix, -95, 0);
    DriveTrainController::driveToPoint(&state, pointSix, -100, -79, 0);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointThree, 100, -90, 1);
    DriveTrainController::turnToPoint(&state, pointSeven, 0, 1);
    DriveTrainController::driveToPoint(&state, pointSeven, 100, 0, 1);
    tilter.set_value(true);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointEight, 0, 0);
    DriveTrainController::driveToPoint(&state, pointEight, -100, 0, 0);
    clamp.set_value(true);
    DriveTrainController::turnToPoint(&state, pointNine, -20, 0);
    DriveTrainController::driveToPoint(&state, pointNine, -100, -87, 0);
    clamp.set_value(false);
}

void test(void* p) {
    Point start(25, 25);
    Point forward(25, 60);
    Point end(15, 25);

    tilter.set_value(true);
    state.switchDir(), state.switchDir();
    DriveTrainController::driveToPoint(&state, forward, -100, 0, 0);
    /*printf("driving");
    DriveTrainController::driveToPoint(&state, forward, -100, 0, 0);
    pros::delay(500);
    printf("turning");
    state.switchDir();
    DriveTrainController::turnToPoint(&state, end, 0, 0);
    DriveTrainController::turnToPoint(&state, end, 0, 0);
    pros::delay(500);
    printf("driving");
    DriveTrainController::driveToPoint(&state, end, 100, 0, 0);
    pros::delay(500);
    printf("turning");

    state.switchDir();
    DriveTrainController::turnToPoint(&state, start, 0, 0);
    DriveTrainController::turnToPoint(&state, start, 0, 0);

    pros::delay(500);
    printf("driving\n");
    DriveTrainController::driveToPoint(&state, start, -100, 0, 0);
    printf("turning\n");
    pros::delay(500);
    DriveTrainController::turnToPoint(&state, forward, 0, 1);
    pros::delay(500);
    printf("driving\n");
    DriveTrainController::driveToPoint(&state, forward, -100, 0, 0);
    pros::delay(500);
    state.switchDir();
    printf("turning\n");

    DriveTrainController::turnToPoint(&state, end, 0, 0);
    DriveTrainController::turnToPoint(&state, end, 0, 0);
    pros::delay(500);
    printf("driving\n");
    DriveTrainController::driveToPoint(&state, end, 100, 0, 0);
    pros::delay(500);

    state.switchDir();
    printf("turning\n");
    DriveTrainController::turnToPoint(&state, start, 0, 0);
    DriveTrainController::turnToPoint(&state, start, 0, 0);

    pros::delay(500);
    printf("turning\n");
    DriveTrainController::driveToPoint(&state, start, -100, 0, 0);
    */
}
void autonomous() {
    //state.switchDir();
    lift.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    pros::Task odomTasks(odomFunctions);
    pros::Task driveTask(test);

}