#include "main.h"
#include "Globals.hpp"
pros::Rotation horEnc(14);
pros::Motor rightFront(8, pros::E_MOTOR_GEARSET_36, 0), rightMid(7, pros::E_MOTOR_GEARSET_36, 0), rightBack(18, pros::E_MOTOR_GEARSET_36, 0),
leftFront(20, pros::E_MOTOR_GEARSET_36, 1), leftMid(1, pros::E_MOTOR_GEARSET_36,1), leftBack(17, pros::E_MOTOR_GEARSET_36, 1), lift(2, pros::E_MOTOR_GEARSET_36, 1), intake(6, pros::E_MOTOR_GEARSET_36, 1);
pros::Controller cont(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut clamp('E', false), tilter('F', false) , cover('D', false), highReleaseL('A', false), highReleaseR('B', false);
pros::Rotation rightEnc(15), leftEnc(13);
pros::Vision vision(16);

pros::Imu inert(11);

pros::vision_signature_s_t RED_SIG =
pros::Vision::signature_from_utility(RED_ID, 8033, 9981, 9007, -1421, -697, -1059, 4.900, 0);
pros::vision_signature_s_t BLUE_SIG =
pros::Vision::signature_from_utility(BLUE_ID, -3473, -2639, -3056, 9517, 11293, 10405, 5.400, 0);
pros::vision_signature_s_t YELLOW_SIG =
pros::Vision::signature_from_utility(YELLOW_ID, 203, 2305, 1254, -5593, -5127, -5360, 4.800, 0);

pros::vision_signature_s_t DARK_YELLOW =
pros::Vision::signature_from_utility(5, 2513, 3101, 2808, -5101, -4715, -4908, 4.500, 1);
pros::vision_signature_s_t LIGHT_YELLOW =
pros::Vision::signature_from_utility(6, -1, 597, 298, -5973, -5419, -5696, 3.100, 1);
pros::vision_color_code_t blred =
vision.pros::Vision::create_color_code(RED_ID,BLUE_ID);
bool inAuton = true;