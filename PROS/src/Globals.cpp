#include "main.h"
pros::ADIEncoder rightEnc('C', 'D', true), leftEnc('A', 'B', true), horEnc('E', 'F', true);
pros::Motor rightFront(10, 1), rightBack(9, 1), leftFront(1, 0), leftBack(2, 0), intake(19, 1), frontLift(21, 1);
pros::Controller cont(pros::E_CONTROLLER_MASTER);