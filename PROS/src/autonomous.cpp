#include "main.h"
#include "917Classes/DriveTrainState.hpp"
#include <algorithm>    // std::max



//DriveTrainState state(110.5, 12, M_PI);
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
        printf("%f\n", std::max(deltaRight, std::max(deltaLeft, deltaMid)));
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
        display.encoderDebug(covRight, "angle to point: ");
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
    DriveTrainController::driveToPoint(&state, pointOne, -100, 0, 10);
    clamp.set_value(true);
    DriveTrainController::driveToPoint(&state, pointTwo, 100, 0, 0);
    DriveTrainController::turnToPoint(&state, pointThree, 0, -10);
    DriveTrainController::driveToPoint(&state, pointThree,100, 0, -10);
    pros::delay(700);

    //DriveTrainController::turnToPoint(&state, pointTwo, .25, -10);
    //DriveTrainController::driveToPoint(&state, pointTwo, 100, .2, -10);
    DriveTrainController::driveToPoint(&state, Point(state.getPos().x - 10, fabs(atan(state.getTheta())) * 10 + state.getPos().y), -100, 220, -10);
    DriveTrainController::turnToPoint(&state, pointFive, 250, -10);
    DriveTrainController::driveToPoint(&state, pointFive, 100, 250, -10);
    pros::delay(2000);
    DriveTrainController::turnToPoint(&state, pointSix, 250, -10);

}

void testing(void* p) {
    Point pointOne(25, 60);
    Point pointTwo(25, 25);
    Point pointThree(0, 45);
    Point pointFour(50, 45);
    Point pointFive(0, 0);
    Point pointSix(50, 0);
    printf("%f\n", Utils::perToVol(100));
    //bident.move_voltage(Utils::perToVol(100));
    
    /*
    DriveTrainController::turnToPoint(&state, pointThree, 0, 0);
    pros::delay(1000);
    DriveTrainController::turnToPoint(&state, pointFour, 0, 0);
    pros::delay(1000);
    DriveTrainController::turnToPoint(&state, pointThree, 0, 0);
    pros::delay(1000);
    DriveTrainController::turnToPoint(&state, pointSix, 0, 0);
    pros::delay(1000);
    DriveTrainController::turnToPoint(&state, pointFive, 0, 0);
    pros::delay(1000);
    DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    
    */

    DriveTrainController::driveToPoint(&state, pointOne, 100, 0, 0);
    //DriveTrainController::driveToPoint(&state, pointTwo, 100, 0, 0);
    //DriveTrainController::turnToPoint(&state, pointThree, 0, 0);
    //DriveTrainController::driveToPoint(&state, pointThree, 100, 0, 0);
    //pros::delay(700);

    //DriveTrainController::turnToPoint(&state, pointTwo, .25, -10);
    //DriveTrainController::driveToPoint(&state, pointTwo, 100, .2, -10);
    //DriveTrainController::driveToPoint(&state, Point(state.getPos().x - 10, fabs(atan(state.getTheta())) * 10 + state.getPos().y), -100, 220, -10);
    //DriveTrainController::turnToPoint(&state, pointFive, 250, 0);
    //DriveTrainController::driveToPoint(&state, pointFive, 100, 250, 0);
    //bident.move_absolute(250 * FORK_RATIO, 100);
    //pros::delay(2000);
    //DriveTrainController::turnToPoint(&state, pointSix, 250, -10);
    

}
void autonomous() {
    pros::Task odomTasks(odomFunctions);
    pros::Task driveTask(testing);

}