#include "DriveTrainController.hpp"
#include "DriveTrainState.hpp"
#include "Utils.hpp"
#include "Globals.hpp"
#include <math.h>



	void DriveTrainController::turnToPoint(DriveTrainState * state, Point target) {
		double pointAng = Utils::angleToPoint(Point(target.x-state->getPos().x, target.y-state->getPos().y));
		double targetAng = pointAng - state->getTheta();
		double angNeeded = targetAng;
		if (targetAng > M_PI / 2) targetAng -= M_PI;
		else if (targetAng < -M_PI / 2) targetAng += M_PI;

		while (fabs(targetAng) > minErrorDegrees) {
			double pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
			double targetAng = pointAng - state->getTheta();
			double angNeeded = targetAng;
			double spd;
			if (targetAng > M_PI / 2) targetAng -= M_PI;
			else if (targetAng < -M_PI / 2) targetAng += M_PI;
			if (fabs(targetAng) < AngleUntilDecelerate) {
				spd = linSpd;
			}
			else {
				//https://www.desmos.com/calculator/nzlh716nfy
				spd = (oSpeed - linSpd) / 2 * (1 + cos((M_PI-fabs(targetAng)))) + linSpd;
			}
			int coefficient = 1;
			if (targetAng < 0) {
				coefficient *= -1;
			}

			rightBack.move_voltage(Utils::perToVol(spd) * coefficient);
			rightFront.move_voltage(Utils::perToVol(spd) * coefficient);
			leftBack.move_voltage(Utils::perToVol(spd) * -coefficient);
			leftFront.move_voltage(Utils::perToVol(spd) * -coefficient);
			pros::delay(20);

		}

	};

