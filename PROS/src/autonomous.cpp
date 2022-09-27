#include "main.h"
#include "917Classes/DriveTrainState.hpp"
#include "917Classes/structDefs.hpp"
#include <algorithm>    // std::max
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <unordered_map>

//hashmap to organize various starting positions
std::unordered_map<std::string, std::vector<double>> StartMap = {   { "right neutral",std::vector<double>{110.5, 18, 0}},
                                                                    { "left neutral",std::vector<double>{24, 18, 0}},
                                                                    { "right mid",std::vector<double>{103.5, 12, 0}},
                                                                    { "skills",std::vector<double>{27, 13.5, M_PI / 2 - .1463}},
                                                                    {"test",std::vector<double>{0,0,0}} };


std::string key = "skills";
DriveTrainState state(StartMap.at(key)[0], StartMap.at(key)[1], StartMap.at(key)[2]);


int intakeState = 2;
int progState = 0;
double inertHeading = state.getTheta();



double kInert = 1080.0 / (1080 + 12);


//Thread for state control using sensor information
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
    double prevTheta = theta;
    inert.set_yaw(-180.0/M_PI*(state.getTheta()));
    //inertHeading = state.getTheta();

    Point pointTwo(0, 0);
    while (1) {
        bool aPress = cont.get_digital_new_press(DIGITAL_LEFT);
        if (aPress) {
            printf("current: %f, %f, %f \n", state.getPos().x, state.getPos().y, state.getTheta());
        }
        //double pointAng = Utils::angleToPoint(Point(pointTwo.x - state.getPos().x, pointTwo.y - state.getPos().y));
        //double targetAng = pointAng - state.getTheta();
        //printf("%f, %f\n", covRight, covLeft);
        pros::delay(20);


        //ensures more data is collected if sensors have not rotated enough
        if (std::max(fabs(deltaRight), std::max(fabs(deltaLeft), fabs(deltaMid))) < DriveTrainState::minTicks) {
            covRight = rightEnc.get_position() / 100, covLeft = leftEnc.get_position() / 100, covMid = horEnc.get_position()/100;
            deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
            printf("charging\n");
            continue;
        }
        //printf("not charging\n");
        
        prevTheta = theta;
        theta = -Utils::inertToRad(inert.get_yaw());
        double dTheta = (theta - prevTheta) * kInert;

        if (dTheta >= M_PI) {
            dTheta -= 2 * M_PI * kInert;
        }
        else if (dTheta <= -M_PI) {
            dTheta += 2 * M_PI * kInert;
        }
        //printf("theta: %f, dTheta: %f, inertialheading: %f \n", theta,  dTheta, inertHeading);
        inertHeading = Utils::thetaConverter(inertHeading + dTheta);
        state.step(deltaLeft, deltaRight, deltaMid, dTheta);
        
        prevRight = covRight, prevLeft = covLeft, prevMid = covMid;
        covRight = rightEnc.get_position() / 100.0, covLeft = leftEnc.get_position() / 100.0, covMid = horEnc.get_position()/100.0;
        deltaRight = covRight - prevRight, deltaLeft = covLeft - prevLeft, deltaMid = covMid - prevMid;
    }
}

//thread for working with Brain Screen, option for vision sensor debug or for visual position tracking
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

//Thread to communicate between reseting class and DriveTrainController class
//reset the position of the robot according to parameters in reseter, and changes a pointer
//to tell the DriveTrainController to exit its current function
void mogoReseter(void* reseter) {

    resetStruct* activeRester = static_cast<resetStruct*>(reseter);
    activeRester->posReset = false;
    activeRester->switchPressed = false;
    while (!activeRester->posReset) {
        pros::delay(20);

        if (limitSwitch.get_value() && !activeRester->switchPressed&&!activeRester->posReset) {
            activeRester->switchPressed = true;
            state.setState(Utils::mogoReset(activeRester->pointReset, inertHeading), inertHeading);

        }

    }

    activeRester->posReset = false;
}
void rightSide(void* p) {
    lift.tare_position();
    resetStruct nothing(Point(0,0));
    DriveTrainController controller(&state, &nothing);
    Point pointOne(110.5, 66.01);
    Point pointTwo(110, 35);
    Point pointThree(140, 41);
    Point pointFour(130.5, 39);
    Point pointFive(125, 78);
    Point pointSix(125, 35);
    Point pointSeven(124, 5);
    Point pointEight(110, 25);
    Point pointNine(110, 0);
    //controller.turnToPoint( , pointOne, 0, 0);
    cover.set_value(true);
    controller.driveToPoint(pointOne, -100, 0,2, true);
    clamp.set_value(true);
    //pros::delay(200);
    printf("looping through motor");

    state.switchDir();
    controller.driveToPoint(pointTwo, 100, 0,2,true);

    controller.turnToPoint(pointThree, -10, 1);
    cover.set_value(false);
    clamp.set_value(false);
    controller.driveToPoint(pointThree,100, 0, 0,  true);
    tilter.set_value(true);
    state.switchDir();
    //controller.turnToPoint(pointFour, -20, 0);
    //controller.driveToPoint(pointFour, -100, -20, 0);
    intakeState = 0;
    //intake.move(-127);
    controller.driveToPoint(pointFive, -70, -0, 0, -6, false);
    state.switchDir();
    controller.driveToPoint(pointSix, 100, -0, 0,  false);
    state.switchDir();
    controller.driveToPoint(pointSeven, -80, 0, 0,  false);
    state.setState(Point(110,10),M_PI);
    state.switchDir();
    controller.driveToPoint(pointEight, 100, -0, 0,  false);
    pros::delay(1000);
    state.switchDir();
    controller.driveToPoint(pointNine, -30, -0, 0,  false);
    tilter.set_value(false);
    highReleaseR.set_value(true);
    highReleaseL.set_value(true);

    intakeState = 3;
    
}

void midTow(void* p) {

    resetStruct nothing(Point(0,0));
    DriveTrainController controller(&state, &nothing);
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
    //controller.turnToPoint(pointOne, 0, 0);
    cover.set_value(true);
    controller.driveToPoint(pointOne, -100, 0, 2,  true);
    clamp.set_value(true);
    state.switchDir();
    controller.driveToPoint(Point(state.getPos().x + 15, state.getPos().y- 15/fabs(tan(state.getTheta())) ), 100, -10, 0,  false);
    controller.turnToPoint(pointThree, 0, 0);
    clamp.set_value(false);
    cover.set_value(false);
    controller.driveToPoint(pointThree, 100, 0, 0,  true);
    tilter.set_value(true);
    state.switchDir();
    //controller.turnToPoint(pointFour, -20, 0);
    //controller.driveToPoint(pointFour, -100, -20, 0);
    //controller.turnToPoint(pointFive, 0, 0);
    intake.move(-127);
    //controller.driveToPoint(pointFive, -70, -0, 0, -6, false);
    //state.switchDir();
    controller.driveToPoint(pointSeven, -40, -0, 0, -16, false);
    //state.switchDir();
    //controller.driveToPoint(pointSeven, -60, 0, -4, 0, false);
    state.setState(Point(110, 10), M_PI);
    state.switchDir();
    controller.driveToPoint(pointEight, 100, -0, 0,  false);
    pros::delay(1000);
    state.switchDir();
    controller.driveToPoint(pointNine, -30, -0, 0,  false);
    highReleaseR.set_value(true);
    highReleaseL.set_value(true);

    //pros::delay(700);
    intakeState = 3;
}
void leftSide(void* p) {

    resetStruct nothing(Point(0,0));
    DriveTrainController controller(&state, &nothing);

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
    controller.driveToPoint(pointOne, -100, 0, 2,  true);
    clamp.set_value(false);
    state.switchDir();
    controller.driveToPoint(pointTwo, 100, 0, 2,  false);
    controller.turnToPoint(pointThree, -15, 0);
    cover.set_value(false);
    clamp.set_value(false);
    controller.driveToPoint(pointThree, 70, 0, 0,  true);
    intake.move(-127);

    tilter.set_value(true);
    state.switchDir();
    controller.driveToPoint(pointFour, -70, 0, 0,  false);
    intake.move(-127);
    
}
void prog(void* p) {
    int speed = 70;
    lift.tare_position();
    Point pointZero(28, 13);
    Point pointOne(26.6, 69.5);
    Point pointTwoOh(65.5, 120);
    Point pointThree(64.5, 111);
    Point pointFour(120, 114);
    Point pointFive(91, 145);
    Point pointFiveOh(107, 131);
    Point pointSix(107, 80.5);
    Point pointSeven(76.5, 124);
    Point pointEight(84, 113);
    Point pointNine(70.5, 78);
    Point pointTen(69.5, 124);
    Point pointEleven(74, 100);
    Point pointTwelve(144, 58);
    Point pointTwelveOh(100, 60);
    Point pointThirteen(70, 109);
    Point pointFourteen(70, 99);
    Point pointFifteen(45, 94);
    Point pointSixteen(60, 124);
    Point pointSeventeen(41, 110);
    Point pointEighteen(0, 87);
    Point pointEighteenOh(19, 87);

    Point pointNineteen(50, -3);
    Point balance(72, 12);


    Point yelLeft(36, 72);
    Point yelMid(72, 72);
    Point yelRight(108, 72);
    Point blue(132, 36);
    Point tester(50, 98);
    Point redLeft(12, 108);



    tilter.set_value(true);
    pros::delay(100);
    resetStruct resetInterface(yelLeft);
    DriveTrainController controller(&state, &resetInterface);
    //pros::delay(1000);

    pros::Task resetThread(mogoReseter, (void*)&resetInterface);
    controller.driveToPoint(pointOne, -speed, 6, 0,  true);
    clamp.set_value(true);
    intakeState = 0;
    //intake.move(-127);
    //pros::delay(500);

    controller.driveToPoint(pointTwoOh, -50, -90, 0, 10, false);
    lift.move_absolute(Utils::redMotConv(-50)*LIFT_RATIO, 100);
    pros::delay(1000);
    clamp.set_value(false);
    
    state.switchDir();
    controller.driveToPoint(pointThree, speed, -65, 0,  false);
    state.switchDir();
    intakeState = 2;
    intake.move(0);
    controller.turnToPoint(pointFour, -65, 0);
    tilter.set_value(false);
    pros::delay(200);
    controller.driveToPoint(pointFour, -speed, -65, 1, false);
    state.switchDir();
    controller.driveToPoint(pointFive, 70, -10, 1,  true);
    tilter.set_value(true);
    state.switchDir();

    controller.driveToPoint(pointFiveOh, -70, -10, 0,  false);

    intakeState = 0;
    //intake.move(-127);
    resetInterface = resetStruct(yelRight);
    pros::Task resetThread2(mogoReseter, (void*)&resetInterface);
    progState = 2;
    controller.driveToPoint(pointSix, -speed, 3, 0,  110, 30, 1, true);
    clamp.set_value(true);
    
    controller.driveToPoint(pointSeven, -60, -65, 0,  false);
    clamp.set_value(false);
    state.switchDir();
    
    controller.driveToPoint(pointEight, speed, -65, 0,  false);
    state.switchDir();
    
    controller.turnToPoint(pointNine, 0, 0);
    resetInterface = resetStruct(yelMid);
    pros::Task resetThread3(mogoReseter, (void*)&resetInterface);
    controller.driveToPoint(pointNine, -50, 2, 0,  110, 30, 1, true);
    clamp.set_value(true);
    
    controller.turnToPoint(pointTen, -60, 0);
    controller.driveToPoint(pointTen, -speed, -67, 0, -14, false);
    lift.move_absolute(Utils::redMotConv(-60) * LIFT_RATIO, 100);
    pros::delay(750);
    clamp.set_value(false);
    state.switchDir();
    controller.driveToPoint(pointEleven, speed, -70, 0,  110, 10, 1, false);
    state.switchDir();
    controller.turnToPoint(pointTwelve, -15, 0);
    resetInterface = resetStruct(blue);
    pros::Task resetThread4(mogoReseter, (void*)&resetInterface);
    controller.driveToPoint(pointTwelve, -speed, 0, 0, -72, 110, 57, 1, true);
    clamp.set_value(true);
    state.switchDir();
    controller.driveToPoint(pointTwelveOh, speed, -10, 0,  false);
    state.switchDir();
    controller.driveToPoint(pointThirteen, -speed, -86, 0,  110, -1, 1, false);
    clamp.set_value(false);
    state.switchDir();
    controller.driveToPoint(pointFourteen, speed, -80, 0,  false);
    state.switchDir();
    resetInterface = resetStruct(tester);
    pros::Task resetThread5(mogoReseter, (void*)&resetInterface);
    controller.driveToPoint(pointFifteen, -speed, 0, 0,  false);
    clamp.set_value(true);
    controller.driveToPoint(pointSixteen, -speed, -88, 0,  false);
    clamp.set_value(false);
    state.switchDir();
    controller.driveToPoint(pointSeventeen, speed, -75, 0,  false);
    state.switchDir();
    resetInterface = resetStruct(redLeft);
    pros::Task resetThread6(mogoReseter, (void*)&resetInterface);
    controller.driveToPoint(pointEighteen, -speed, 0, 0,  true);
    clamp.set_value(true);
    state.switchDir();
    controller.driveToPoint(pointEighteenOh, speed, 0, 0,  true);
    state.switchDir();
    controller.driveToPoint(pointNineteen, -100, -90, 0,  true);
    //controller.driveToPoint(balance, -100, 0, 2,  110, 2, 0, false);
}

void leftWPFirst(void* p) {

    resetStruct nothing(Point(0,0));
    DriveTrainController controller(&state, &nothing);

    int speed = 70;
    lift.tare_position();
    Point pointZero(28, 13);
    Point pointOne(33, 72);
    Point pointTwoOh(25, 10);

    //printf("%f\n", Utils::perToVol(100));
    //state.switchDir();
    //controller.driveToPoint(pointZero, speed, 6, 1,  true);

    tilter.set_value(true);
    //controller.turnToPoint(pointOne, 0, 0);
    pros::delay(100);
    //state.switchDir();

    //pros::delay(1000);
    controller.driveToPoint(pointOne, -speed, 6, 0,  true);
    clamp.set_value(true);
    intakeState = 0;
    intake.move(-127);
    //pros::delay(500);
    state.switchDir();

    controller.driveToPoint(pointTwoOh, speed, -10, 0,  true);


}

void test(void* p) {
    lift.tare_position();
    Point forward(0, 25);

    Point climb(25, 75);
    //intake.move(127);
    //state.switchDir();
    resetStruct resetInterface(climb);
    DriveTrainController controller(&state, &resetInterface);
    tilter.set_value(true);

    pros::Task resetThread(mogoReseter, (void*)&resetInterface);
    controller.driveToPoint(forward, -20, 6, 0,  true);
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
    void* shoot = nullptr;
    lift.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
    pros::Task odomTasks(odomFunctions);
    pros::Task displayTask(visualDebug, &debugTaskName);
    //pros::Task activeRester(mogoReseter, shoot);
    pros::Task driveTask(prog);
    //pros::Task autonIntakeTask(autonIntake);

}


