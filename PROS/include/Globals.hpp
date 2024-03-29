#include "main.h"
#pragma once
extern pros::Rotation rightEnc;
extern pros::Rotation leftEnc;
extern pros::Rotation horEnc;
extern pros::Motor rightFront;
extern pros::Motor rightMid;
extern pros::Motor rightBack;
extern pros::Motor leftFront;
extern pros::Motor leftMid;
extern pros::Motor leftBack;
extern pros::Motor intake;
extern pros::Motor lift;
extern pros::ADIDigitalOut clamp;
extern pros::ADIDigitalOut tilter;
extern pros::ADIDigitalOut highReleaseL;
extern pros::ADIDigitalOut highReleaseR;
extern pros::Vision vision;

extern pros::ADIDigitalIn limitSwitch;

extern pros::Controller cont;
extern pros::ADIDigitalOut cover;
extern pros::Imu inert;
extern pros::vision_signature_s_t RED_SIG;
extern pros::vision_signature_s_t BLUE_SIG;
extern pros::vision_signature_s_t YELLOW_SIG;

extern pros::vision_signature_s_t DARK_YELLOW;
extern pros::vision_signature_s_t LIGHT_YELLOW;
extern pros::vision_color_code_t blred;
extern bool inAuton;