#include "main.h"
pros::ADIEncoder rightEnc('C', 'D'), leftEnc('E', 'F', true), horEnc('G', 'H');
pros::Imu inert(20);