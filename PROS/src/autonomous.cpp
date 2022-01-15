#include "main.h"
#define _USE_MATH_DEFINES



double inToDeg(double inch) {
    return inch * 360 / M_PI / 4;
}
void drive(double driveLength, double speed) {
    driveLength = inToDeg(driveLength);
    rightFront.move_relative(driveLength, speed);
    rightMid.move_relative(driveLength, speed);
    rightBack.move_relative(driveLength, speed);
    leftFront.move_relative(driveLength, speed);
    leftMid.move_relative(driveLength, speed);
    leftBack.move_relative(driveLength, speed);
}
void turn(double driveLength, double speed) {
    driveLength = inToDeg(driveLength);
    rightFront.move_relative(driveLength, speed);
    rightMid.move_relative(driveLength, speed);
    rightBack.move_relative(driveLength, speed);
    leftFront.move_relative(driveLength, -speed);
    leftMid.move_relative(driveLength, -speed);
    leftBack.move_relative(driveLength, -speed);
}
void mogoRight() {
    double bidentRot = 3500;
    clamp.set_value(false);
    drive(-40, 600);
    pros::delay(2000);
    clamp.set_value(true);
    drive(10, 300);
    turn(10, 300);
    drive(10, 600);
    pros::delay(1000);
    frontLift.move_relative(bidentRot*5/6, 600);
    drive(-10, 600);
    pros::delay(1000);
    frontLift.move_relative(bidentRot / 6, 600);
    turn(-3, 200);
    drive(10, 600);
    pros:delay(1000);
    frontLift.move_relative(-bidentRot, 600);
    pros::delay(1000);
    drive(-10, 600);
}

void winPoint() {

}
void autonomous() {
    
    
}