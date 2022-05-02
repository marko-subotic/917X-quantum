#include "main.h"
#include "917Classes/DriveTrainState.hpp"
#include <algorithm>    // std::max
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */

//right neutral auton
//DriveTrainState state(110.5, 12, 0);
//right mid auton
//DriveTrainState state(103.5, 12, 0);
//for skills
DriveTrainState state(27, 13.5, M_PI/2-.1463);
// for testing
//DriveTrainState state(0, 0, 0);

void odomFunctions(void* p) {
    rightEnc.reset_position();
    if (!rightEnc.get_reversed()) {
        rightEnc.reverse();
    }
    leftEnc.reset_position();
    if(!horEnc.get_reversed()){
        horEnc.reverse();
    }
    horEnc.reset_position();
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
    double theta = state.getTheta();
    display.setState(state.getPos(), theta);
    Point pointTwo(0, 0);
    while (1) {
        bool aPress = cont.get_digital_new_press(DIGITAL_LEFT);
        if (aPress) {
            printf("current: %f, %f, %f \n", state.getPos().x, state.getPos().y, state.getTheta());
        }
        //double pointAng = Utils::angleToPoint(Point(pointTwo.x - state.getPos().x, pointTwo.y - state.getPos().y));
        //double targetAng = pointAng - state.getTheta();
        //printf("%f, %f\n", covRight, covLeft);

        //printf("not charging\n");

        state.step(deltaLeft, deltaRight, deltaMid);
        theta = state.getTheta();
        display.setState(state.getPos(), theta);
        display.encoderDebug(covRight, "angle to point: ");
        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.get_position() / 100.0, covLeft = leftEnc.get_position() / 100.0, covMid = horEnc.get_position()/100.0;
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
        pros::delay(20);
    }
}

void prog(void* p) {
    int speed = 70;
    lift.tare_position();
    Point pointOne(35, 72);
    Point pointTwoOh(62, 117);
    Point pointThree(53, 108);
    Point pointFour(127, 112);
    Point pointFive(99, 137);
    Point pointSix(108.5, 80);
    Point pointSeven(82, 120);
    Point pointEight(80, 113);
    Point pointNine(68, 75);
    Point pointTen(67, 121);
    Point pointEleven(67, 113);
    //printf("%f\n", Utils::perToVol(100));

    tilter.set_value(true);
    //DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    pros::delay(100);

    //pros::delay(1000);
    DriveTrainController::driveToPoint(&state, pointOne, -100, 6, 0, ANGLE_IRRELEVANT, true);
    clamp.set_value(true);
    intake.move(-127);

    //pros::delay(500);

    DriveTrainController::driveToPoint(&state, pointTwoOh, -speed, -100, 0, 29, false);
    lift.move_absolute(Utils::redMotConv(-61)*LIFT_RATIO, 100);
    pros::delay(500);
    clamp.set_value(false);
    
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointThree, 100, -65, 0, ANGLE_IRRELEVANT, false);
    state.switchDir();
    tilter.set_value(false);
    
    DriveTrainController::driveToPoint(&state, pointFour, -100, -65, 0, 83, false);
    state.switchDir();
    
    DriveTrainController::driveToPoint(&state, pointFive, 60, -100, 1, ANGLE_IRRELEVANT, true);
    tilter.set_value(true);
    state.switchDir();
    
    DriveTrainController::driveToPoint(&state, pointSix, -100, 3, 0, ANGLE_IRRELEVANT, true);
    clamp.set_value(true);
    
    DriveTrainController::driveToPoint(&state, pointSeven, -speed, -72, 0, -40, false);
    clamp.set_value(false);
    state.switchDir();
    
    DriveTrainController::driveToPoint(&state, pointEight, 100, -70, 0, ANGLE_IRRELEVANT, false);
    state.switchDir();
    
    DriveTrainController::turnToPoint(&state, pointNine, 0, 0);
    DriveTrainController::driveToPoint(&state, pointNine, -100, 5, 0, ANGLE_IRRELEVANT, true);
    clamp.set_value(true);
    
    DriveTrainController::turnToPoint(&state, pointTen, -75, 0);
    DriveTrainController::driveToPoint(&state, pointTen, -speed, -72, 0, -3, false);
    lift.move_absolute(Utils::redMotConv(-57) * LIFT_RATIO, 100);
    pros::delay(750);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointEleven, 100, -75, 0, -7, 110, 17, 0, false);
    

   
}

void test(void* p) {
    lift.tare_position();
    Point forward(20.5, 7);

    Point climb(25, 75);
    //intake.move(127);
    //state.switchDir();
    //DriveTrainController::driveToPoint(&state, forward, 50, 0, 1, ANGLE_IRRELEVANT);
    //DriveTrainController::driveToPoint(&state, climb, -100, 0, 1, 0);
    tilter.set_value(true);
    DriveTrainController::turnToPoint(&state, forward, 0, 0);
    Point print;
    //intake.move(0);
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