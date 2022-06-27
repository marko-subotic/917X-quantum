#include "main.h"

pros::Rotation horEnc(14);
pros::Motor rightFront(8, pros::E_MOTOR_GEARSET_36, 0), rightMid(7, pros::E_MOTOR_GEARSET_36, 0), rightBack(18, pros::E_MOTOR_GEARSET_36, 0),
leftFront(11, pros::E_MOTOR_GEARSET_36, 1), leftMid(1, pros::E_MOTOR_GEARSET_36,1), leftBack(17, pros::E_MOTOR_GEARSET_36, 1), lift(2, pros::E_MOTOR_GEARSET_36, 1), intake(6, pros::E_MOTOR_GEARSET_36, 1);
pros::Controller cont(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut clamp('E', false), tilter('F', false) , cover('D', false), highReleaseL('A', false), highReleaseR('B', false);
pros::Rotation rightEnc(15), leftEnc(13);
pros::Vision vision(16);

pros::vision_signature_s_t RED_SIG =
pros::Vision::signature_from_utility(RED_ID, 6929, 12233, 9581, -1707, -91, -899, 2.800, 0);
pros::vision_signature_s_t BLUE_SIG =
pros::Vision::signature_from_utility(BLUE_ID, -3399, -1305, -2352, 4617, 12149, 8383, 1.400, 0);
pros::vision_signature_s_t YELLOW_SIG =
pros::Vision::signature_from_utility(YELLOW_ID, 1401, 3747, 2574, -5439, -4681, -5060, 2.100, 0);
bool inAuton = true;