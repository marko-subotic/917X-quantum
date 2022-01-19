#include "main.h"
pros::ADIEncoder rightEnc('C', 'D', true), leftEnc('A', 'B', true), horEnc('E', 'F', true);
pros::Motor rightFront(10, 0), rightBack(9, 0), leftFront(1, 1), leftBack(2, 1), lift(19, 1), bident(21, 1);
pros::Controller cont(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut clamp('G', false);