#include "main.h"
#include "917Classes/DriveTrainState.hpp"



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
    double theta = 0;
    Point pointTwo(0, 0);
    while (1) {
        double pointAng = Utils::angleToPoint(Point(pointTwo.x - state.getPos().x, pointTwo.y - state.getPos().y));
        double targetAng = pointAng - state.getTheta();
        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.get_value(), covLeft = leftEnc.get_value(), covMid = horEnc.get_value();
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
        state.step(deltaLeft, deltaRight, deltaMid);
        theta = state.getTheta();
        display.setState(state.getPos(), theta);
        display.encoderDebug(targetAng, "angle to point: ");
        pros::delay(1);
    }
}

void motorControl(void* p) {
    Point pointOne(0, 0);
    Point pointTwo(25, -15);
    printf("looping through motor");
    //DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    DriveTrainController::driveToPoint(&state, pointTwo, -100, 0, 0);
}
void autonomous() {
    pros::Task odomTasks(odomFunctions);
    pros::Task driveTask(motorControl);

}