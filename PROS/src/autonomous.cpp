#include "main.h"
#include "917Classes/DriveTrainState.hpp"
#include <algorithm>    // std::max
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */

//right neutral auton
//DriveTrainState state(110.5, 12, 0);
//right mid auton
//DriveTrainState state(103.5, 12, 0);
//for skills
DriveTrainState state(27, 10.5, M_PI/2);
// for testing
//DriveTrainState state(25, 25, 0);

void odomFunctions(void* p) {
    rightEnc.reset_position();
    if (!rightEnc.get_reversed()) {
        rightEnc.reverse();
    }
    leftEnc.reset_position();
    horEnc.reset();
    lv_obj_clean(lv_scr_act());
    OdomDisplay display(lv_scr_act());

    double prevRight = 0;
    double prevLeft = 0;
    double prevMid = 0;
    double covRight = rightEnc.get_position()/100;
    double covLeft = leftEnc.get_position()/100;
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
            covRight = rightEnc.get_position()/100, covLeft = leftEnc.get_position()/100, covMid = horEnc.get_value();
            deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
            //printf("charging\n");
            continue;
        }
        //printf("not charging\n");

        state.step(deltaLeft, deltaRight, deltaMid);
        theta = state.getTheta();
        display.setState(state.getPos(), theta);
        display.encoderDebug(deltaLeft, "angle to point: ");
        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.get_position()/100, covLeft = leftEnc.get_position()/100, covMid = horEnc.get_value();
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
        pros::delay(10);
    }
}

void prog(void* p) {
    int speed = 30;
    lift.tare_position();
    Point pointOne(34, 62);
    Point pointTwo(72, 90);
    Point pointTwoOh(68, 116);
    Point pointThree(61, 90);
    Point pointFour(51, 81);
    Point pointFive(70, 97);
    Point pointSix(67, 114);
    Point pointSeven(6, 120);
    Point pointSevenOh(45, 116);
    Point pointEight(70, 75);
    Point pointNine(78, 35.5);
    Point pointNineOh(78, 40);
    Point pointNineDrop(79, 33.5);
    Point pointTen(74, 55);
    Point pointEleven(71, 60);
    Point pointTwelve(74, 42);

    //printf("%f\n", Utils::perToVol(100));

    tilter.set_value(true);
    DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    pros::delay(100);
    intake.move(-127);

    //pros::delay(1000);
    DriveTrainController::driveToPoint(&state, pointOne, -speed, 6, 0,0);
    clamp.set_value(true);
    pros::delay(500);

    DriveTrainController::turnToPoint(&state, pointTwoOh, -100, 0);
    DriveTrainController::driveToPoint(&state, pointTwoOh, -speed, -100, 0, 20);
    lift.move_absolute(Utils::redMotConv(-79), 100);
    pros::delay(500);
    clamp.set_value(false);


    pros::delay(500);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointThree, -79, 0);
    DriveTrainController::driveToPoint(&state, pointThree, speed, -79, 0, 10);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointFive, 0, 1);
    tilter.set_value(false);
    pros::delay(300);
    DriveTrainController::driveToPoint(&state, pointFive, -speed, 0, 1, 20);
    DriveTrainController::turnToPoint(&state, pointFour, -20, 1);
    DriveTrainController::driveToPoint(&state, pointFour, -speed, 4, 1, 40);
    clamp.set_value(true);
    DriveTrainController::turnToPoint(&state, pointSix, -95, 0);
    DriveTrainController::turnToPoint(&state, pointSix, -95, 0);
    DriveTrainController::driveToPoint(&state, pointSix, -speed, -79, 0, 5);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointThree, -90, 1);
    DriveTrainController::turnToPoint(&state, pointThree, -90, 1);

    DriveTrainController::driveToPoint(&state, pointThree, speed, -90, 1, 0);
    pros::delay(300);

    /*DriveTrainController::turnToPoint(&state, pointSevenOh, 0, 1);
    DriveTrainController::turnToPoint(&state, pointSevenOh, 0, 1);
    pros::delay(300);

    DriveTrainController::driveToPoint(&state, pointSevenOh, speed, 0, 1);
    */
    pros::delay(300);

    DriveTrainController::turnToPoint(&state, pointSeven, 0, 1);
    DriveTrainController::turnToPoint(&state, pointSeven, 0, 1);

    pros::delay(300);
    double targetAng = Utils::angleToPoint(Point(pointSeven.x - state.getPos().x, pointSeven.y - state.getPos().y));
    //printf("%f, %f\n", targetAng, state.getTheta());
    DriveTrainController::driveToPoint(&state, pointSeven, speed, 0, 1, 94);
    tilter.set_value(true);
    state.setState(Point(16, 108), M_PI / 2);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointEight, 0, 0);
    DriveTrainController::driveToPoint(&state, pointEight, -speed, 0, 0,-33);
    clamp.set_value(true);
    DriveTrainController::turnToPoint(&state, pointNine, -20, 0);
    DriveTrainController::driveToPoint(&state, pointNine, -speed, -79, 0, 0);

    lift.move_absolute(Utils::redMotConv(-60) * LIFT_RATIO, 100);

    pros::delay(500);
    clamp.set_value(false);
    pros::delay(500);

    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointNineOh, speed, -60, 0, 0);
    DriveTrainController::turnToPoint(&state, pointTen, -65, 0);
    DriveTrainController::driveToPoint(&state, pointTen, speed, -65, 0, 10);
    tilter.set_value(false);
    pros::delay(500);
    state.switchDir();
    //DriveTrainController::turnToPoint(&state, pointTwelve, 0, 1);
    DriveTrainController::driveToPoint(&state, pointTwelve, -speed, 0, 1, 0);

    DriveTrainController::turnToPoint(&state, pointEleven, 0, 1);
    DriveTrainController::driveToPoint(&state, pointEleven, -speed, 0, 1, 15);
    clamp.set_value(true);
    DriveTrainController::turnToPoint(&state, pointNineDrop, -40, 0);
    DriveTrainController::driveToPoint(&state, pointNineDrop, -speed, -79, 0, 0);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointEleven, -79, 0);
    DriveTrainController::driveToPoint(&state, pointEleven, speed, -79, 0, 0);
}

void test(void* p) {
    lift.tare_position();
    Point forward(30, 35);
    intake.move(127);
    DriveTrainController::turnToPoint(&state, forward, 0, 1);
    Point print;
    intake.move(0);
    pros::delay(500);
    double pointAng = Utils::angleToPoint(Point(forward.x - state.getPos().x, forward.y - state.getPos().y));
    double targetAng = pointAng - state.getTheta();
    printf("targetAng: %f \n", targetAng);
   
}
void autonomous() {
    //state.switchDir();
    lift.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    pros::Task odomTasks(odomFunctions);
    pros::Task driveTask(prog);

}