#include "main.h"
pros::ADIEncoder rightEnc('C', 'D'), leftEnc('E', 'F', true), horEnc('G', 'H');
pros::Motor rightFront(10, 1), rightBack(9, 1), leftFront(1, 1), leftBack(2, 1), intake(19, 1), frontLift(21, 1);
pros::Controller cont(pros::E_CONTROLLER_MASTER);