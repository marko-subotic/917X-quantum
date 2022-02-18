#include "main.h"
pros::ADIEncoder rightEnc('G', 'H', false), leftEnc('C', 'D', false), horEnc('A', 'B', false);
pros::Motor rightFront(5, pros::E_MOTOR_GEARSET_36, 0), rightMid(7, pros::E_MOTOR_GEARSET_36, 0), rightBack(18, pros::E_MOTOR_GEARSET_36, 0),
leftFront(4, pros::E_MOTOR_GEARSET_36, 1), leftMid(1, pros::E_MOTOR_GEARSET_36,1), leftBack(17, pros::E_MOTOR_GEARSET_36, 1), lift(10, pros::E_MOTOR_GEARSET_36), intake(6, pros::E_MOTOR_GEARSET_36, 1);
pros::Controller cont(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut clamp('E', false), tilter('F', false);