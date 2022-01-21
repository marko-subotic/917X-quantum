#include "main.h"
pros::ADIEncoder rightEnc('C', 'D'), leftEnc('A', 'B'), horEnc('G', 'H');
pros::Motor rightFront(5, pros::E_MOTOR_GEARSET_06, 0), rightMid(7, pros::E_MOTOR_GEARSET_06, 0), rightBack(18, pros::E_MOTOR_GEARSET_06, 0),
leftFront(4, pros::E_MOTOR_GEARSET_06, 1), leftMid(1, pros::E_MOTOR_GEARSET_06, 1), leftBack(17, pros::E_MOTOR_GEARSET_06, 1), lift(6, 1), bident(10, 1);
pros::Controller cont(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut clamp('E', false);