#include "main.h"
#include "917Classes/DriveTrainState.hpp"
#include <algorithm>    // std::max
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */

//right neutral auton
//DriveTrainState state(110.5, 18, 0);
//right mid auton
//DriveTrainState state(103.5, 12, 0);
//for skills
DriveTrainState state(27, 13.5, M_PI/2-.1463);
//left neutral auton
//DriveTrainState state(24, 18, 0);
// for testing
//DriveTrainState state(0, 0, 0);
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
        
        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.get_position() / 100.0, covLeft = leftEnc.get_position() / 100.0, covMid = horEnc.get_position()/100.0;
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
        pros::delay(20);
    }
}

void visualDebug(void* p) {

    lv_obj_clean(lv_scr_act());
    OdomDisplay display(lv_scr_act());
    display.setState(state.getPos(), state.getTheta());
    while (true) {

        display.update();
        if (display.onVision) {
            display.setVision(vision.get_by_size(0));
        }
        else {
            display.setState(state.getPos(), state.getTheta());
            display.encoderDebug(rightEnc.get_position()/100.0, "angle to point: ");
        }
        pros::delay(20);
    }
}

void centerOfRotation(void* p) {
    lv_obj_clean(lv_scr_act());
    lv_obj_t* container = (lv_obj_create(lv_scr_act(), NULL));
    lv_obj_set_size(container, lv_obj_get_width(lv_scr_act()), lv_obj_get_height(lv_scr_act()));
    lv_obj_align(container, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_style_t* cStyle = lv_obj_get_style(container);
    cStyle->body.main_color = LV_COLOR_BLACK;
    cStyle->body.grad_color = LV_COLOR_BLACK;
    cStyle->body.border.color = LV_COLOR_RED;
    cStyle->body.border.width = 2;
    cStyle->body.radius = 0;

    lv_obj_t* screen = lv_obj_create(container, container);
    double scalar = 1;
    lv_obj_set_size(screen, (int16_t)(VISION_FOV_WIDTH * scalar), (int16_t)(VISION_FOV_HEIGHT * scalar));
    lv_obj_align(screen, container, LV_ALIGN_IN_TOP_LEFT, 0, 0);

    lv_obj_t* label = lv_label_create(container, NULL);
    lv_obj_align(label, container, LV_ALIGN_CENTER, 90, 0);// (int16_t)(lv_obj_get_width(screen) + (lv_obj_get_width(container) - lv_obj_get_width(screen)) / 2), 0);

    lv_style_t* textStyle = lv_obj_get_style(label);
    textStyle->text.color = LV_COLOR_WHITE;
    textStyle->text.opa = LV_OPA_100;
    lv_label_set_text(label, "No Vision\nData Provided");



    pros::vision_object_s_t rtn = vision.get_by_sig(0, RED_ID);

    lv_obj_t* mogo = lv_obj_create(screen, NULL);
    lv_obj_set_size(mogo, (int16_t)(rtn.width * scalar), (int16_t)(rtn.height * scalar));
    lv_obj_set_pos(mogo, (int16_t)(rtn.left_coord * scalar), (int16_t)(rtn.top_coord * scalar));
    lv_style_t mStyle;
    lv_style_copy(&mStyle, cStyle);
    mStyle.body.main_color = LV_COLOR_RED;
    mStyle.body.grad_color = LV_COLOR_RED;
    lv_obj_set_style(mogo, &mStyle);



    lv_style_t lineStyle;
    lv_style_copy(&lineStyle, &lv_style_plain);
    lineStyle.line.width = 1;
    lineStyle.line.color = LV_COLOR_WHITE;
    lv_style_t axisStyle;
    lv_style_copy(&axisStyle, &lineStyle);
    axisStyle.line.width = 3;

    int dashCount = 13;
    int dashWidth = 10;
    lv_point_t startPoint = { 0,0 };
    std::vector<lv_point_t> pointVector(2, startPoint);
    static std::vector<std::vector<lv_point_t>> vertLinePoints(dashCount - 1, pointVector);
    static std::vector<std::vector<lv_point_t>> horLinePoints(dashCount - 1, pointVector);
    int sWidth = lv_obj_get_width(screen);
    int sHeight = lv_obj_get_height(screen);

    lv_obj_t* xAxis = lv_line_create(screen, NULL);
    lv_point_t xAxisPoints[2];
    xAxisPoints[0].x = 0, xAxisPoints[0].y = sHeight / 2-1;
    xAxisPoints[1].x = sWidth, xAxisPoints[1].y = sHeight / 2-1;
    lv_line_set_points(xAxis, xAxisPoints, 2);
    lv_obj_set_style(xAxis, &axisStyle);

    lv_obj_t* yAxis = lv_line_create(screen, NULL);
    lv_point_t yAxisPoints[2];
    yAxisPoints[0].y = 0, yAxisPoints[0].x = sWidth / 2-1;
    yAxisPoints[1].y = sHeight, yAxisPoints[1].x = sWidth / 2-1;
    lv_line_set_points(yAxis, yAxisPoints, 2);
    lv_obj_set_style(yAxis, &axisStyle);

    for (int i = 1; i < dashCount; i++) {
        vertLinePoints[i - 1][0].x = (int16_t)(sWidth / dashCount * i);
        vertLinePoints[i - 1][0].y = (int16_t)(sHeight / 2 + dashWidth / 2);
        vertLinePoints[i - 1][1].x = (int16_t)(sWidth / dashCount * i);
        vertLinePoints[i - 1][1].y = (int16_t)(sHeight / 2 - dashWidth / 2);
        lv_obj_t* line = lv_line_create(screen, NULL);
        lv_line_set_points(line, vertLinePoints[i - 1].data(), vertLinePoints[i - 1].size());
        lv_obj_set_free_num(line, i);

        lv_obj_set_style(line, &lineStyle);
    }
    for (int i = 1; i < dashCount; i++) {
        horLinePoints[i - 1][0].x = (int16_t)(sWidth / 2 + dashWidth / 2);
        horLinePoints[i - 1][0].y = (int16_t)(sHeight / dashCount * i);
        horLinePoints[i - 1][1].x = (int16_t)(sWidth / 2 - dashWidth / 2);
        horLinePoints[i - 1][1].y = (int16_t)(sHeight / dashCount * i);
        lv_obj_t* line = lv_line_create(screen, NULL);
        lv_line_set_points(line, horLinePoints[i - 1].data(), horLinePoints[i - 1].size());
        lv_obj_set_free_num(line, i);

        lv_obj_set_style(line, &lineStyle);
    }

    lv_obj_set_style(mogo, &mStyle);
    lv_obj_set_size(mogo, (int16_t)(10), (int16_t)(10));

    int widthScale = 20;
    int heightScale = sHeight / sWidth * widthScale;
    while (true) {
        pros::delay(20);
        //lv_label_set_text(label, rtn.signature);
        //rtn = vision.get_by_size(0);
        Point COR = state.getCOR();
        
        std::string labelText = std::to_string(COR.x) + "," + std::to_string(COR.y);
        lv_label_set_text(label, labelText.c_str());
        lv_obj_set_pos(mogo, (int16_t)((widthScale/2+COR.x) * sWidth/widthScale), (int16_t)((heightScale/2 - COR.y) * sHeight / heightScale ));
        lv_obj_invalidate(mogo);
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
    Point pointSeven(77, 124);
    Point pointEight(80, 113);
    Point pointNine(71, 75);
    Point pointTen(60, 125);
    Point pointEleven(68, 100);
    Point pointTwelve(150, 51);
    Point pointTwelveOh(124, 70);
    Point pointThirteen(65, 120);
    Point pointFourteen(66, 102);
    Point pointFifteen(40, 88);
    Point pointSixteen(45, 117);
    Point pointSeventeen(41, 98);
    Point pointEighteen(0, 87);
    Point pointEighteenOh(19, 87);

    Point pointNineteen(50, -3);
    Point balance(72, 12);
    //printf("%f\n", Utils::perToVol(100));
    //state.switchDir();
    //DriveTrainController::driveToPoint(&state, pointZero, speed, 6, 0, ANGLE_IRRELEVANT, true);

    tilter.set_value(true);
    //DriveTrainController::turnToPoint(&state, pointOne, 0, 0);
    pros::delay(100);
    //state.switchDir();

    //pros::delay(1000);
    DriveTrainController::driveToMogo(&state, pointOne, -speed, 6, 0, ANGLE_IRRELEVANT, true, YELLOW_ID);
    clamp.set_value(true);
    intakeState = 0;
    intake.move(-127);
    //pros::delay(500);

    DriveTrainController::driveToPoint(&state, pointTwoOh, -50, -90, 0, 10, false);
    lift.move_absolute(Utils::redMotConv(-50)*LIFT_RATIO, 100);
    pros::delay(1000);
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
    
    DriveTrainController::driveToMogo(&state, pointSix, -speed, 3, 0, ANGLE_IRRELEVANT, 110, 70, 1, true, YELLOW_ID);
    clamp.set_value(true);
    
    DriveTrainController::driveToPoint(&state, pointSeven, -60, -68, 0, ANGLE_IRRELEVANT, false);
    clamp.set_value(false);
    state.switchDir();
    
    DriveTrainController::driveToPoint(&state, pointEight, speed, -65, 0, ANGLE_IRRELEVANT, false);
    state.switchDir();
    
    DriveTrainController::turnToPoint(&state, pointNine, 0, 0);
    DriveTrainController::driveToMogo(&state, pointNine, -speed, 2, 0, ANGLE_IRRELEVANT, 110, 80, 1, true, YELLOW_ID);
    clamp.set_value(true);
    
    DriveTrainController::turnToPoint(&state, pointTen, -60, 0);
    DriveTrainController::driveToPoint(&state, pointTen, -speed, -60, 0, ANGLE_IRRELEVANT, false);
    lift.move_absolute(Utils::redMotConv(-60) * LIFT_RATIO, 100);
    pros::delay(750);
    clamp.set_value(false);
    state.switchDir();
    DriveTrainController::driveToPoint(&state, pointEleven, speed, -70, 0, ANGLE_IRRELEVANT, 110, 10, 0, false);
    state.switchDir();
    DriveTrainController::turnToPoint(&state, pointTwelve, -15, 0);
    DriveTrainController::driveToMogo(&state, pointTwelve, -speed, 0, 0, -72, 110, 57, 1, true, BLUE_ID);
clamp.set_value(true);
state.switchDir();
DriveTrainController::driveToPoint(&state, pointTwelveOh, speed, -10, 0, ANGLE_IRRELEVANT, false);
state.switchDir();
DriveTrainController::driveToPoint(&state, pointThirteen, -speed, -86, 0, ANGLE_IRRELEVANT, 110, 10, 0, false);
clamp.set_value(false);
state.switchDir();
DriveTrainController::driveToPoint(&state, pointFourteen, speed, -80, 0, ANGLE_IRRELEVANT, false);
state.switchDir();
DriveTrainController::driveToMogo(&state, pointFifteen, -speed, 0, 0, ANGLE_IRRELEVANT, false, BLUE_ID);
clamp.set_value(true);
DriveTrainController::driveToPoint(&state, pointSixteen, -speed, -88, 0, ANGLE_IRRELEVANT, false);
clamp.set_value(false);
state.switchDir();
DriveTrainController::driveToPoint(&state, pointSeventeen, speed, -65, 0, ANGLE_IRRELEVANT, false);
state.switchDir();
DriveTrainController::driveToMogo(&state, pointEighteen, -speed, 0, 0, ANGLE_IRRELEVANT, true, RED_ID);
clamp.set_value(true);
state.switchDir();
DriveTrainController::driveToMogo(&state, pointEighteenOh, speed, 0, 0, ANGLE_IRRELEVANT, true, RED_ID);
state.switchDir();
DriveTrainController::driveToPoint(&state, pointNineteen, -100, -90, 0, ANGLE_IRRELEVANT, true);
//DriveTrainController::driveToPoint(&state, balance, -100, 0, 2, ANGLE_IRRELEVANT, 110, 2, 0, false);
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
    Point forward(0, 25);

    Point climb(25, 75);
    //intake.move(127);
    //state.switchDir();
    tilter.set_value(true);

    //DriveTrainController::driveToPoint(&state, forward, -100, 0, 0, 0, false);
    //DriveTrainController::driveToPoint(&state, climb, -100, 0, 1, 0);
    
    DriveTrainController::driveToMogo(&state, forward, -70, 0, 0, ANGLE_IRRELEVANT, false, RED_ID);
    //DriveTrainController::driveToPoint(&state, forward, -70, 0, 0, ANGLE_IRRELEVANT, true);
    Point print;
    //intake.move(0);
    pros::delay(500);
    //double pointAng = Utils::angleToPoint(Point(forward.x - state.getPos().x, forward.y - state.getPos().y));
    //double targetAng = pointAng - state.getTheta();
    //printf("targetAng: %f \n", targetAng);

}

void autonIntake(void* p) {
    DriveTrainController::intakeTask(&intakeState);
}
void autonomous() {
    //state.switchDir();
    std::string debugTaskName("visualDebug Task");

    lift.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    pros::Task odomTasks(odomFunctions);
    //pros::Task displayTask(visualDebug, &debugTaskName);
    pros::Task bruh(centerOfRotation);
    //pros::Task driveTask(prog);
    //pros::Task autonIntakeTask(autonIntake);

}


/*
void autonomous() {


    pros::vision_signature_s_t RED_SIG =
        pros::Vision::signature_from_utility(2, 7677, 12161, 9919, -1239, -363, -801, 2.600, 0);
    vision.set_signature(2, &RED_SIG);
    pros::vision_object_s_t rtn = vision.get_by_sig(0, 2);
    printf("%d\n", rtn.width);
    int counter = 0;
    while (vision.get_object_count()>0) {
        rtn = vision.get_by_sig(0, 2);
        int place = rtn.left_coord + rtn.width/2 - 158;
        printf("%d\n", rtn.width);
        pros::delay(20);

        if (rtn.width > 210&&abs(place)<18) {
            counter++;
        }
        else {
            counter = 0;
        }
        if (counter > 3) {
            break;
        }
        place *= .25;
        int spd = -30;
        int correction = (rtn.left_coord + rtn.width / 2 - 158) * .25;

        rightBack.move(Utils::perToVol(spd - correction));
        rightMid.move(Utils::perToVol(spd - correction));
        rightFront.move(Utils::perToVol(spd - correction));

        leftMid.move(Utils::perToVol(spd + correction));
        leftBack.move(Utils::perToVol(spd + correction));
        leftFront.move(Utils::perToVol(spd + correction));
    }
    rightBack.move_velocity(0);
    rightFront.move_velocity(0);
    rightMid.move_velocity(0);

    leftMid.move_velocity(0);
    leftBack.move_velocity(0);
    leftFront.move_velocity(0);
}*/

/*
void autonomous(){

    lv_obj_clean(lv_scr_act());
    lv_obj_t* container = (lv_obj_create(lv_scr_act(), NULL));
    lv_obj_set_size(container, lv_obj_get_width(lv_scr_act()), lv_obj_get_height(lv_scr_act()));
    lv_obj_align(container, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_style_t* cStyle = lv_obj_get_style(container);
    cStyle->body.main_color = LV_COLOR_BLACK;
    cStyle->body.grad_color = LV_COLOR_BLACK;
    cStyle->body.border.color = LV_COLOR_RED;
    cStyle->body.border.width = 2;
    cStyle->body.radius = 0;

    lv_obj_t* screen = lv_obj_create(container, container);
    double scalar = 1;
    lv_obj_set_size(screen, (int16_t)(VISION_FOV_WIDTH*scalar), (int16_t)(VISION_FOV_HEIGHT*scalar));
    lv_obj_align(screen, container, LV_ALIGN_IN_TOP_LEFT, 0, 0);

    lv_obj_t* label = lv_label_create(container, NULL);
    lv_obj_align(label, container, LV_ALIGN_CENTER, 90, 0);// (int16_t)(lv_obj_get_width(screen) + (lv_obj_get_width(container) - lv_obj_get_width(screen)) / 2), 0);

    lv_style_t* textStyle = lv_obj_get_style(label);
    textStyle->text.color = LV_COLOR_WHITE;
    textStyle->text.opa = LV_OPA_100;
    lv_label_set_text(label, "No Vision\nData Provided");

    vision.set_signature(RED_ID, &RED_SIG);
    vision.set_signature(BLUE_ID, &BLUE_SIG);
    vision.set_signature(YELLOW_ID, &YELLOW_SIG);

    pros::vision_object_s_t rtn = vision.get_by_sig(0, RED_ID);

    lv_obj_t* mogo = lv_obj_create(screen, NULL);
    lv_obj_set_size(mogo, (int16_t)(rtn.width * scalar), (int16_t)(rtn.height * scalar));
    lv_obj_set_pos(mogo, (int16_t)(rtn.left_coord * scalar), (int16_t)(rtn.top_coord * scalar));
    lv_style_t mStyle;
    lv_style_copy(&mStyle, cStyle);
    mStyle.body.main_color = LV_COLOR_RED;
    mStyle.body.grad_color = LV_COLOR_RED;
    lv_obj_set_style(mogo, &mStyle);
    while (true) {
        pros::delay(20);
        //lv_label_set_text(label, rtn.signature);
        rtn = vision.get_by_size(0);
        printf("%d\n", rtn.signature);
        ///*
        if (rtn.signature == RED_ID) {
            mStyle.body.main_color = LV_COLOR_RED;
            mStyle.body.grad_color = LV_COLOR_RED;
            lv_label_set_text(label, "mogo is red");


        }
        else if (rtn.signature == BLUE_ID) {
            mStyle.body.main_color = LV_COLOR_BLUE;
            mStyle.body.grad_color = LV_COLOR_BLUE;
            lv_label_set_text(label, "mogo is blue");

        }
        else if(rtn.signature == YELLOW_ID){
            mStyle.body.main_color = LV_COLOR_YELLOW;
            mStyle.body.grad_color = LV_COLOR_YELLOW;
            lv_label_set_text(label, "mogo is yellow");


        }
        
        lv_obj_set_style(mogo, &mStyle);
        lv_obj_set_size(mogo, (int16_t)(rtn.width * scalar), (int16_t)(rtn.height * scalar));
        lv_obj_set_pos(mogo, (int16_t)(rtn.left_coord*scalar), (int16_t)(rtn.top_coord*scalar));
        lv_obj_invalidate(mogo);
    }
   
}*/