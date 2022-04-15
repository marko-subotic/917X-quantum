#include "main.h"

pros::ADIEncoder horEnc('A', 'B', false);
pros::Motor rightFront(8, pros::E_MOTOR_GEARSET_36, 0), rightMid(7, pros::E_MOTOR_GEARSET_36, 0), rightBack(18, pros::E_MOTOR_GEARSET_36, 0),
leftFront(11, pros::E_MOTOR_GEARSET_36, 1), leftMid(1, pros::E_MOTOR_GEARSET_36,1), leftBack(17, pros::E_MOTOR_GEARSET_36, 1), lift(2, pros::E_MOTOR_GEARSET_36), intake(6, pros::E_MOTOR_GEARSET_36, 1);
pros::Controller cont(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut clamp('E', false), tilter('F', false) , cover('D', false);
pros::Rotation rightEnc(15), leftEnc(4);
bool inAuton = true;