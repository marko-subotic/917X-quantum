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
//DriveTrainState state(25, 10.5, 3*M_PI/2);

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
    int speed = 50;
    lift.tare_position();
    Point pointOne(34, 62);
    Point pointTwoOh(63, 116);
    Point pointThree(61, 114);
    Point pointFour(129, 110);
    Point pointFive(111, 133);
    Point pointSix(110.5, 87);
    Point pointSeven(85, 118);
    Point pointEight(80, 113);
    Point pointNine(79, 84);
    Point pointTen(73, 121);
    Point pointEleven(74, 113);
    //printf("%f\n", Utils::perToVol(100));

    tilter.set_value(true);
    //DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    pros::delay(100);

    //pros::delay(1000);
    DriveTrainController::driveToPoint(&state, pointOne, -100, 6, 0,5);
    clamp.set_value(true);
    intake.move(-127);

    //pros::delay(500);

    DriveTrainController::driveToPoint(&state, pointTwoOh, -100, -100, 0, 29);
    lift.move_absolute(Utils::redMotConv(-61)*LIFT_RATIO, 100);
    pros::delay(500);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointThree, 100, -65, 0, Utils::angleBetweenPoints(pointThree, pointTwoOh));
    state.switchDir();
    tilter.set_value(false);
    DriveTrainController::driveToPoint(&state, pointFour, -100, -65, 0, 84);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointFive, 50, -65, 1, -80);
    tilter.set_value(true);
    state.switchDir();
    /*DriveTrainController::driveToPoint(&state, pointSix, -100, 3, 0, -6);
    clamp.set_value(true);
    DriveTrainController::driveToPoint(&state, pointSeven, -70, -79, 0, -39);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointEight, 100, -70, 0, -29);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointNine, 0, 0);
    DriveTrainController::driveToPoint(&state, pointNine, -100, 0, 0, 3);
    clamp.set_value(true);
    DriveTrainController::turnToPoint(&state, pointTen, -64, 0);
    DriveTrainController::driveToPoint(&state, pointTen, -100, -75, 0, -10);
    lift.move_absolute(Utils::redMotConv(-57) * LIFT_RATIO, 100);
    pros::delay(750);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointEleven, 100, -75, 0, -7, 110, 20);
    */

   
}

void test(void* p) {
    lift.tare_position();
    Point forward(40, 10.5);
    Point climb(76, 10.5);
    //intake.move(127);
    lift.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    pros::delay(2000);
    clamp.set_value(true);
    double currentLift = lift.get_position();
    pros::delay(2000);
    lift.move_absolute(currentLift, 100);
    
    
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