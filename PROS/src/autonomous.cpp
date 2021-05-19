#include "main.h"

void autonomous() {
    pros::delay(100);
    lv_obj_clean(lv_scr_act());
    OdomDebug display(lv_scr_act());
    DriveTrainState state = DriveTrainState(50, 50, 0);
    double prevRight = 0;
    double prevLeft = 0;
    double prevMid = 0;
    double prevTheta = 0;
    double covRight = rightEnc.get_value();
    double covLeft = leftEnc.get_value();
    double covMid = horEnc.get_value();
    double covTheta = inert.get_heading();
    double deltaRight = covRight-prevRight;
    double deltaLeft = covLeft-prevLeft;
    double deltaMid = covMid-prevMid;
    double deltaTheta = covTheta - prevTheta;
    double theta = inert.get_yaw();
    while (1) {
        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.get_value(), covLeft = leftEnc.get_value(), covMid = horEnc.get_value();
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
        prevTheta = covTheta;
        covTheta = inert.get_heading();
        deltaTheta = covTheta - prevTheta;
        theta = inert.get_yaw();
        state.step(deltaLeft, deltaRight, deltaMid, deltaTheta, theta);
        //display.setState(state.getPos(), theta);
        pros::delay(20);
    }
    
}