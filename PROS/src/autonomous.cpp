#include "main.h"

void autonomous() {
    lv_obj_clean(lv_scr_act());
    OdomDisplay display(lv_scr_act());
    double prevRight = 0;
    double prevLeft = 0;
    double prevMid = 0;
    double covRight = rightEnc.get_value();
    double covLeft = leftEnc.get_value();
    double covMid = horEnc.get_value();
    double deltaRight = covRight-prevRight;
    double deltaLeft = covLeft-prevLeft;
    double deltaMid = covMid - prevMid;
    double theta = 0;
    DriveTrainState state = DriveTrainState(25, 25, theta);
    while (1) {
        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.get_value(), covLeft = leftEnc.get_value(), covMid = horEnc.get_value();
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid-prevMid;
        state.step(deltaLeft, deltaRight, deltaMid);
        theta = state.getTheta();
        display.setState(state.getPos(), theta);
        //display.encoderDebug(covMid, "right: encoder");
        pros::delay(20);
    }
    
}