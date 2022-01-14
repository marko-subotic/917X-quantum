#include "main.h"

void autonomous() {
    
    double driveLength = 20;
    rightFront.move_relative(driveLength, 600);
    rightMid.move_relative(driveLength, 600);
    rightBack.move_relative(driveLength, 600);
    leftFront.move_relative(driveLength, 600);
    leftMid.move_relative(driveLength, 600);
    leftBack.move_relative(driveLength, 600);
    clamp.set_value(true);
    rightFront.move_relative(-driveLength, 600);
    rightMid.move_relative(-driveLength, 600);
    rightBack.move_relative(-driveLength, 600);
    leftFront.move_relative(-driveLength, 600);
    leftMid.move_relative(-driveLength, 600);
    leftBack.move_relative(-driveLength, 600);
}