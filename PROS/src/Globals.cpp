#include "main.h"
//pros::ADIEncoder rightEnc('C', 'D', true), leftEnc('A', 'B', true), horEnc('E', 'F', true);
pros::Motor rightFront(5, pros::E_MOTOR_GEARSET_06, 0), rightMid(7, pros::E_MOTOR_GEARSET_06, 0), rightBack(18, pros::E_MOTOR_GEARSET_06, 0), 
leftFront(4, pros::E_MOTOR_GEARSET_06, 1), leftMid(1, pros::E_MOTOR_GEARSET_06, 1), leftBack(17, pros::E_MOTOR_GEARSET_06, 1), 
intake(10, pros::E_MOTOR_GEARSET_06), frontLift(6, pros::E_MOTOR_GEARSET_06, 1);
pros::Controller cont(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut clamp('B', false);