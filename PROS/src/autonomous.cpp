#include "main.h"
#include "917Classes/DriveTrainState.hpp"
#include <algorithm>    // std::max
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */

//right neutral auton
//DriveTrainState state(110.5, 12, 0);
//right mid auton
//DriveTrainState state(103.5, 12, 0);
//for skills
//DriveTrainState state(27, 10.5, M_PI/2);
// for testing
DriveTrainState state(25, 25, 0);

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

void rightSide(void* p) {

    Point pointOne(110.5, 55);
    Point pointTwo(110, 35);
    Point pointThree(130, 39);
    Point pointFour(130.5, 39);
    Point pointFive(132, 78);
    Point pointSix(125, 30);
    //DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    DriveTrainController::driveToPoint(&state, pointOne, -100, 0,1);
    clamp.set_value(true);
    printf("looping through motor");

    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointTwo, 100, -20,0);
    /*DriveTrainController::turnToPoint(&state, pointThree, -20, 0);
    DriveTrainController::turnToPoint(&state, pointThree, -20, 0);
    DriveTrainController::driveToPoint(&state, pointThree,100, -10, 0);
    tilter.set_value(true);
    state.switchDir();
    //DriveTrainController::turnToPoint(&state, pointFour, -20, 0);
    //DriveTrainController::driveToPoint(&state, pointFour, -100, -20, 0);
    DriveTrainController::turnToPoint(&state, pointFive, -30, 0);
    intake.move(-127);
    DriveTrainController::driveToPoint(&state, pointFive, -30, -30, 0);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointSix, -20, 0);
    DriveTrainController::driveToPoint(&state, pointSix, 100, -20, 0);
    */
    //pros::delay(700);

    //DriveTrainController::turnToPoint(&state, pointTwo, .25, -10);
    //DriveTrainController::driveToPoint(&state, pointTwo, 100, .2, -10);
    //DriveTrainController::driveToPoint(&state, Point(state.getPos().x - 10, fabs(atan(state.getTheta())) * 10 + state.getPos().y), -100, Utils::redMotConv(-10),0);
    /*DriveTrainController::turnToPoint(&state, pointFive, Utils::redMotConv(-10), 0);
    DriveTrainController::driveToPoint(&state, pointFive, 100, Utils::redMotConv(-10),0);
    pros::delay(2000);
    DriveTrainController::turnToPoint(&state, pointSix, Utils::redMotConv(-10),0);
    */
}

void midTow(void* p) {
    Point pointOne(67, 55.25);
    Point pointTwo(82, 55.25);
    Point pointThree(122, 44);
    Point pointFour(130.5, 39);
    Point pointFive(115, 78);
    Point pointSix(125, 30);
    Point statePos = state.getPos();
    printf("looping through motor");
    state.setState(statePos, Utils::angleToPoint(Point(pointOne.x - statePos.x, pointOne.y - statePos.y)));
    //DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    DriveTrainController::driveToPoint(&state, pointOne, -100, 0, 1);
    clamp.set_value(true);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, Point(state.getPos().x + 15, state.getPos().y- 15/fabs(tan(state.getTheta())) ), 100, -10, 0);
    DriveTrainController::turnToPoint(&state, pointThree, -10, 0);
    DriveTrainController::turnToPoint(&state, pointThree, -10, 0);
    DriveTrainController::driveToPoint(&state, pointThree, 100, -10, 0);
    tilter.set_value(true);
    state.switchDir();
    //DriveTrainController::turnToPoint(&state, pointFour, -20, 0);
    //DriveTrainController::driveToPoint(&state, pointFour, -100, -20, 0);
    DriveTrainController::turnToPoint(&state, pointFive, -30, 0);
    intake.move(-127);
    DriveTrainController::driveToPoint(&state, pointFive, -30, -30, 0);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointSix, -20, 0);
    DriveTrainController::driveToPoint(&state, pointSix, 100, -20, 0);

    //pros::delay(700);

    //DriveTrainController::turnToPoint(&state, pointTwo, .25, -10);
    //DriveTrainController::driveToPoint(&state, pointTwo, 100, .2, -10);
    //DriveTrainController::driveToPoint(&state, Point(state.getPos().x - 10, fabs(atan(state.getTheta())) * 10 + state.getPos().y), -100, Utils::redMotConv(-10),0);
    /*DriveTrainController::turnToPoint(&state, pointFive, Utils::redMotConv(-10), 0);
    DriveTrainController::driveToPoint(&state, pointFive, 100, Utils::redMotConv(-10),0);
    pros::delay(2000);
    DriveTrainController::turnToPoint(&state, pointSix, Utils::redMotConv(-10),0);
    */
}

void prog(void* p) {
    
    lift.tare_position();
    Point pointOne(35, 62);
    Point pointTwo(66, 112);
    Point pointThree(65, 94);
    Point pointFour(62, 90);
    Point pointFive(70, 105);
    Point pointSix(70, 107.5);
    Point pointSeven(26.75, 116);
    Point pointEight(66, 78);
    Point pointNine(64, 33.5);
    Point pointTen(64, 45);
    Point pointEleven(64, 37);

    //printf("%f\n", Utils::perToVol(100));
    
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


    pros::delay(500);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointThree, -79, 0);
    DriveTrainController::driveToPoint(&state, pointThree, 100, -79, 0);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointFive, 0, 1);
    tilter.set_value(false);
    DriveTrainController::driveToPoint(&state, pointFive, -100, 0, 1);
    DriveTrainController::turnToPoint(&state, pointFour, -20, 1);
    DriveTrainController::driveToPoint(&state, pointFour, -100, 4, 1);
    clamp.set_value(true);
    DriveTrainController::turnToPoint(&state, pointSix, -95, 0);
    DriveTrainController::turnToPoint(&state, pointSix, -95, 0);
    DriveTrainController::driveToPoint(&state, pointSix, -100, -79, 0);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointThree, -90, 1);
    DriveTrainController::driveToPoint(&state, pointThree, 100, -90, 1);
    pros::delay(500);
    DriveTrainController::turnToPoint(&state, pointSeven, 0, 1);
    DriveTrainController::driveToPoint(&state, pointSeven, 100, 0, 1);
    tilter.set_value(true);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointEight, 0, 0);
    DriveTrainController::driveToPoint(&state, pointEight, -100, 0, 0);
    clamp.set_value(true);
    DriveTrainController::turnToPoint(&state, pointNine, -20, 0);
    DriveTrainController::driveToPoint(&state, pointNine, -90, -76, 0);
    clamp.set_value(false);
    pros::delay(500);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointTen, -76, 0);
    DriveTrainController::driveToPoint(&state, pointTen, 100, -83, 0);
    tilter.set_value(false);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointNine, 0, 1);
    DriveTrainController::driveToPoint(&state, pointNine, -100, 0, 1);

    DriveTrainController::turnToPoint(&state, pointEleven, 0, 1);
    DriveTrainController::driveToPoint(&state, pointEleven, -100, 0, 1);
    clamp.set_value(true);
    DriveTrainController::turnToPoint(&state, pointNine, -40, 0);
    DriveTrainController::driveToPoint(&state, pointNine, -100, -79, 0);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointEleven, -79, 0);
    DriveTrainController::driveToPoint(&state, pointEleven, 100, -79, 0);
}

void test(void* p) {
    lift.tare_position();
    Point start(25, 25);
    Point forward(25, 50);
    Point end(12, 37.5);

    Point pointTwo(66, 112);
    Point pointThree(65, 94);
    Point pointFour(62, 90);
    Point pointFive(70, 105);
    Point pointSix(70, 107.5);
   tilter.set_value(true);

    DriveTrainController::turnToPoint(&state, end, 0, 0);
    pros::delay(100);
    DriveTrainController::driveToPoint(&state, end, -100, 0, 0);
    /*state.switchDir();
    state.setState(pointTwo, Utils::angleToPoint(Point(pointThree.x-pointTwo.x, pointThree.y-pointTwo.y)));
    DriveTrainController::turnToPoint(&state, pointThree, -79, 0);
    DriveTrainController::driveToPoint(&state, pointThree, 100, -79, 0);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointFive, 0, 1);
    tilter.set_value(false);
    DriveTrainController::driveToPoint(&state, pointFive, -100, 0, 1);
    DriveTrainController::turnToPoint(&state, pointFour, -20, 1);
    DriveTrainController::driveToPoint(&state, pointFour, -100, 4, 1);
    clamp.set_value(true);
    //DriveTrainController::turnToPoint(&state, pointSix, -95, 0);
    DriveTrainController::turnToPoint(&state, pointSix, -95, 0);
    DriveTrainController::driveToPoint(&state, pointSix, -100, -79, 0);
    clamp.set_value(false);
    */
    //tilter.set_value(true);
    //DriveTrainController::turnToPoint(&state, forward, 0, 0);
    //DriveTrainController::turnToPoint(&state, forward, 0, 0);
    //clamp.set_value(true);
    //DriveTrainController::driveToPoint(&state, forward, -100, -10, 0);
    //DriveTrainController::turnToPoint(&state, end, -10, 0);
    //DriveTrainController::driveToPoint(&state, end, -100, -10, 0);
    
}
void autonomous() {
    //state.switchDir();
    lift.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    pros::Task odomTasks(odomFunctions);
    pros::Task driveTask(test);

}