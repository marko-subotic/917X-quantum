#include "main.h"
#include "917Classes/DriveTrainState.hpp"
#include <algorithm>    // std::max
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */

//right neutral auton
//DriveTrainState state(110.5, 18, 0);
//right mid auton
//DriveTrainState state(103.5, 12, 0);
//for skills
//DriveTrainState state(27, 13.5, M_PI/2-.1463);
//left neutral auton
//DriveTrainState state(24, 18, 0);
// for testing
DriveTrainState state(0, 0, 0);
int intakeState = 2;

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

void rightSide(void* p) {
    lift.tare_position();
    Point pointOne(110.5, 66.01);
    Point pointTwo(110, 35);
    Point pointThree(140, 41);
    Point pointFour(130.5, 39);
    Point pointFive(125, 78);
    Point pointSix(125, 35);
    Point pointSeven(124, 5);
    Point pointEight(110, 25);
    Point pointNine(110, 0);
    //DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    cover.set_value(true);
    DriveTrainController::driveToPoint(&state, pointOne, -100, 0,2, ANGLE_IRRELEVANT, true);
    clamp.set_value(true);
    //pros::delay(200);
    printf("looping through motor");

    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointTwo, 100, 0,2, ANGLE_IRRELEVANT, true);

    DriveTrainController::turnToPoint(&state, pointThree, -10, 1);
    cover.set_value(false);
    clamp.set_value(false);
    DriveTrainController::driveToPoint(&state, pointThree,100, 0, 0, ANGLE_IRRELEVANT, true);
    tilter.set_value(true);
    state.switchDir();
    //DriveTrainController::turnToPoint(&state, pointFour, -20, 0);
    //DriveTrainController::driveToPoint(&state, pointFour, -100, -20, 0);
    intakeState = 0;
    intake.move(-127);
    DriveTrainController::driveToPoint(&state, pointFive, -70, -0, 0, -6, false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointSix, 100, -0, 0, ANGLE_IRRELEVANT, false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointSeven, -80, 0, 0, ANGLE_IRRELEVANT, false);
    state.setState(Point(110,10),M_PI);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointEight, 100, -0, 0, ANGLE_IRRELEVANT, false);
    pros::delay(1000);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointNine, -30, -0, 0, ANGLE_IRRELEVANT, false);
    tilter.set_value(false);
    highReleaseR.set_value(true);
    highReleaseL.set_value(true);

    //pros::delay(700);
    intakeState = 3;
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
    Point pointOne(61, 60);
    Point pointTwo(82, 55.25);
    Point pointThree(130, 42);
    Point pointFour(130.5, 39);
    Point pointFive(119, 78);
    Point pointSix(119, 35);
    Point pointSeven(119, 5);
    Point pointEight(110, 25);
    Point pointNine(110, 0);
    Point statePos = state.getPos();
    printf("looping through motor");
    state.setState(statePos, Utils::angleToPoint(Point(pointOne.x - statePos.x, pointOne.y - statePos.y)));
    //DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    cover.set_value(true);
    DriveTrainController::driveToPoint(&state, pointOne, -100, 0, 2, ANGLE_IRRELEVANT, true);
    clamp.set_value(true);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, Point(state.getPos().x + 15, state.getPos().y- 15/fabs(tan(state.getTheta())) ), 100, -10, 0, ANGLE_IRRELEVANT, false);
    DriveTrainController::turnToPoint(&state, pointThree, 0, 0);
    clamp.set_value(false);
    cover.set_value(false);
    DriveTrainController::driveToPoint(&state, pointThree, 100, 0, 0, ANGLE_IRRELEVANT, true);
    tilter.set_value(true);
    state.switchDir();
    //DriveTrainController::turnToPoint(&state, pointFour, -20, 0);
    //DriveTrainController::driveToPoint(&state, pointFour, -100, -20, 0);
    //DriveTrainController::turnToPoint(&state, pointFive, 0, 0);
    intake.move(-127);
    //DriveTrainController::driveToPoint(&state, pointFive, -70, -0, 0, -6, false);
    //state.switchDir();
    DriveTrainController::driveToPoint(&state, pointSeven, -40, -0, 0, -16, false);
    //state.switchDir();
    //DriveTrainController::driveToPoint(&state, pointSeven, -60, 0, -4, 0, false);
    state.setState(Point(110, 10), M_PI);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointEight, 100, -0, 0, ANGLE_IRRELEVANT, false);
    pros::delay(1000);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointNine, -30, -0, 0, ANGLE_IRRELEVANT, false);
    highReleaseR.set_value(true);
    highReleaseL.set_value(true);

    //pros::delay(700);
    intakeState = 3;
}
void leftSide(void* p) {
    Point pointOne(35, 69);
    Point pointTwo(26, 30);
    Point pointThree(47.5, 14);
    Point pointFour(35, 36);
    Point pointFive(24, 5);
    Point pointSix(24, 20);
    cover.set_value(true);
    Point statePos = state.getPos();
    clamp.set_value(true);
    state.setState(statePos, Utils::angleToPoint(Point(pointOne.x - statePos.x, pointOne.y - statePos.y)));
    DriveTrainController::driveToPoint(&state, pointOne, -100, 0, 2, ANGLE_IRRELEVANT, true);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointTwo, 100, 0, 2, ANGLE_IRRELEVANT, false);
    DriveTrainController::turnToPoint(&state, pointThree, -15, 0);
    cover.set_value(false);
    clamp.set_value(false);
    DriveTrainController::driveToPoint(&state, pointThree, 70, 0, 0, ANGLE_IRRELEVANT, true);
    intake.move(-127);

    tilter.set_value(true);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointFour, -70, 0, 0, ANGLE_IRRELEVANT, false);
    intake.move(-127);
    /*DriveTrainController::driveToPoint(&state, pointFive, -70, -0, 0, ANGLE_IRRELEVANT, false);
    state.switchDir();
    state.setState(Point(24, 5), M_PI);

    //tilter.set_value(false);
    DriveTrainController::driveToPoint(&state, pointSix, 100, -0, 0, ANGLE_IRRELEVANT, false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointFive, -80, 0, 0, ANGLE_IRRELEVANT, false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointSix, 100, -0, 0, ANGLE_IRRELEVANT, false);
    pros::delay(2000);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointFive, -30, -0, 0, ANGLE_IRRELEVANT, false);
    highReleaseL.set_value(true);
    highReleaseR.set_value(true);
    //pros::delay(700);
    intakeState = 3;*/
}
void prog(void* p) {
    int speed = 70;
    lift.tare_position();
    Point pointZero(28, 13);
    Point pointOne(33, 72);
    Point pointTwoOh(62, 118);
    Point pointThree(60, 109);
    Point pointFour(120, 112);
    Point pointFive(87, 144);
    Point pointSix(104.5, 80);
    Point pointSeven(71, 124);
    Point pointEight(80, 113);
    Point pointNine(69, 75);
    Point pointTen(59, 125);
    Point pointEleven(68, 100);
    Point pointTwelve(150, 51);
    Point pointTwelveOh(124, 70);
    Point pointThirteen(60, 120);
    Point pointFourteen(66, 102);
    Point pointFifteen(40, 80);
    Point pointSixteen(45, 117);
    Point pointSeventeen(41, 98);
    Point pointEighteen(0, 87);
    Point pointNineteen(50, 10);
    Point balance(72, 12);
    //printf("%f\n", Utils::perToVol(100));
    //state.switchDir();
    //DriveTrainController::driveToPoint(&state, pointZero, speed, 6, 0, ANGLE_IRRELEVANT, true);

    tilter.set_value(true);
    //DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    pros::delay(100);
    //state.switchDir();

    //pros::delay(1000);
    DriveTrainController::driveToPoint(&state, pointOne, -speed, 6, 0, ANGLE_IRRELEVANT, true);
    clamp.set_value(true);
    intakeState = 0;
    intake.move(-127);
    //pros::delay(500);

    DriveTrainController::driveToPoint(&state, pointTwoOh, -50, -90, 0, 10, false);
    lift.move_absolute(Utils::redMotConv(-50)*LIFT_RATIO, 100);
    pros::delay(500);
    clamp.set_value(false);
    
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointThree, speed, -65, 0, ANGLE_IRRELEVANT, false);
    state.switchDir();
    tilter.set_value(false);
    intakeState = 2;
    intake.move(0);
    DriveTrainController::driveToPoint(&state, pointFour, -speed, -65, 1, 83, false);
    state.switchDir();
    pros::delay(500);
    DriveTrainController::driveToPoint(&state, pointFive, 70, -10, 1, ANGLE_IRRELEVANT, true);
    tilter.set_value(true);
    intakeState = 0;
    intake.move(-127);
    state.switchDir();
    
    DriveTrainController::driveToPoint(&state, pointSix, -speed, 3, 0, ANGLE_IRRELEVANT, 110, 70, 1, true);
    clamp.set_value(true);
    
    DriveTrainController::driveToPoint(&state, pointSeven, -60, -68, 0, ANGLE_IRRELEVANT, false);
    clamp.set_value(false);
    state.switchDir();
    
    DriveTrainController::driveToPoint(&state, pointEight, speed, -65, 0, ANGLE_IRRELEVANT, false);
    state.switchDir();
    
    DriveTrainController::turnToPoint(&state, pointNine, 0, 0);
    DriveTrainController::driveToPoint(&state, pointNine, -speed, 2, 0, ANGLE_IRRELEVANT, 110, 80, 1, true);
    clamp.set_value(true);
    
    DriveTrainController::turnToPoint(&state, pointTen, -75, 0);
    DriveTrainController::driveToPoint(&state, pointTen, -speed, -66, 0, ANGLE_IRRELEVANT, false);
    lift.move_absolute(Utils::redMotConv(-63) * LIFT_RATIO, 100);
    pros::delay(750);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointEleven, speed, -70, 0, ANGLE_IRRELEVANT, 110, 10, 0, false);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointTwelve, -15, 0);
    DriveTrainController::driveToPoint(&state, pointTwelve, -speed, 0, 0, -72, 110, 57, 1, true);
clamp.set_value(true);
state.switchDir();
DriveTrainController::driveToPoint(&state, pointTwelveOh, speed, -10, 0, ANGLE_IRRELEVANT, false);
state.switchDir();
DriveTrainController::driveToPoint(&state, pointThirteen, -speed, -86, 0, ANGLE_IRRELEVANT, 110, 10, 0, false);
clamp.set_value(false);
state.switchDir();
DriveTrainController::driveToPoint(&state, pointFourteen, speed, -80, 0, ANGLE_IRRELEVANT, false);
state.switchDir();
DriveTrainController::driveToPoint(&state, pointFifteen, -speed, 0, 0, ANGLE_IRRELEVANT, false);
clamp.set_value(true);
DriveTrainController::driveToPoint(&state, pointSixteen, -speed, -88, 0, ANGLE_IRRELEVANT, false);
clamp.set_value(false);
state.switchDir();
DriveTrainController::driveToPoint(&state, pointSeventeen, speed, -65, 0, ANGLE_IRRELEVANT, false);
state.switchDir();
DriveTrainController::driveToPoint(&state, pointEighteen, -speed, 0, 0, ANGLE_IRRELEVANT, true);
clamp.set_value(true);
DriveTrainController::driveToPoint(&state, pointNineteen, -speed, -90, 0, ANGLE_IRRELEVANT, true);
DriveTrainController::driveToPoint(&state, balance, -100, 0, 2, ANGLE_IRRELEVANT, 110, 2, 0, false);
}

void leftWPFirst(void* p) {
    int speed = 70;
    lift.tare_position();
    Point pointZero(28, 13);
    Point pointOne(33, 72);
    Point pointTwoOh(25, 10);

    //printf("%f\n", Utils::perToVol(100));
    //state.switchDir();
    //DriveTrainController::driveToPoint(&state, pointZero, speed, 6, 1, ANGLE_IRRELEVANT, true);

    tilter.set_value(true);
    //DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    pros::delay(100);
    //state.switchDir();

    //pros::delay(1000);
    DriveTrainController::driveToPoint(&state, pointOne, -speed, 6, 0, ANGLE_IRRELEVANT, true);
    clamp.set_value(true);
    intakeState = 0;
    intake.move(-127);
    //pros::delay(500);
    state.switchDir();

    DriveTrainController::driveToPoint(&state, pointTwoOh, speed, -10, 0, ANGLE_IRRELEVANT, true);


}

void test(void* p) {
    lift.tare_position();
    Point forward(0, 24);

    Point climb(25, 75);
    //intake.move(127);
    //state.switchDir();
    tilter.set_value(true);

    //DriveTrainController::driveToPoint(&state, forward, -100, 0, 0, 0, false);
    //DriveTrainController::driveToPoint(&state, climb, -100, 0, 1, 0);
    
    DriveTrainController::driveToMogo(&state, forward, -70, 0, 0, 0, false, RED_ID);
    Point print;
    //intake.move(0);
    pros::delay(500);
    //double pointAng = Utils::angleToPoint(Point(forward.x - state.getPos().x, forward.y - state.getPos().y));
    //double targetAng = pointAng - state.getTheta();
    printf("targetAng: %f \n", targetAng);

}

void autonIntake(void* p) {
    DriveTrainController::intakeTask(&intakeState);
}
/**/void autonomous() {
    //state.switchDir();
    lift.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    pros::Task odomTasks(odomFunctions);
    pros::Task driveTask(test);
    //pros::Task autonIntakeTask(autonIntake);

}

/*void autonomous() {


    pros::vision_signature_s_t RED_SIG =
        pros::Vision::signature_from_utility(2, 4017, 10127, 7072, -203, 1147, 472, 1.600, 0);
    vision.set_signature(2, &RED_SIG);
    pros::vision_object_s_t rtn = vision.get_by_sig(0, 2);
    while (true) {
        rtn = vision.get_by_sig(0, 2);
        int place = rtn.left_coord + rtn.width/2 - 158;
        printf("%d\n", place);
        pros::delay(100);

        if (abs(place) < 18){
        continue;
        }
        else {
            place *= .25;
            rightBack.move(Utils::perToVol(-place));
            rightMid.move(Utils::perToVol(-place));
            rightFront.move(Utils::perToVol(-place));

            leftMid.move(Utils::perToVol(place));
            leftBack.move(Utils::perToVol(place));
            leftFront.move(Utils::perToVol(place));
        }
    }
}*/