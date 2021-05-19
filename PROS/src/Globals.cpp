#include "main.h"
pros::ADIEncoder rightEnc('C', 'D'), leftEnc('E', 'F', true), horEnc('G', 'H');
pros::Imu inert(20);
const int fieldDimensions = 144;