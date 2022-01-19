#include "DriveTrainController.hpp"
#include "DriveTrainState.hpp"
#include "Utils.hpp"
#include "Globals.hpp"
#include <math.h>



	void DriveTrainController::turnToPoint(DriveTrainState * state, Point target) {
		double pointAng = Utils::angleToPoint(Point(target.x-state->getPos().x, target.y-state->getPos().y));
		double targetAng = pointAng - state->getTheta();
		if (targetAng > M_PI) targetAng -= M_PI;
		else if (targetAng < -M_PI) targetAng += M_PI;

		while (fabs(targetAng) > minErrorDegrees) {
			double pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
			double targetAng = pointAng - state->getTheta();
			double spd;
			if (targetAng > M_PI) targetAng -= M_PI;
			else if (targetAng < -M_PI) targetAng += M_PI;
			if (fabs(targetAng) < AngleUntilLinear) {
				spd = linSpd;
			}if (fabs(targetAng) > M_PI-AngleUntilDecelerate) {
				spd = oSpeed;
			}
			else {
				//https://www.desmos.com/calculator/frano6ozhv
				spd = (oSpeed - linSpd) / 2 * (1 + cos(M_PI/(M_PI-AngleUntilDecelerate)*(M_PI-AngleUntilDecelerate-fabs(targetAng)))) + linSpd;
			}
			int coefficient = 1;
			if (targetAng < 0) {
				coefficient *= -1;
			}
			printf("%f", targetAng);
			printf(", %f\n", spd);

			rightBack.move_voltage(Utils::perToVol(spd) * coefficient);
			rightFront.move_voltage(Utils::perToVol(spd) * coefficient);
			leftBack.move_voltage(Utils::perToVol(spd) * -coefficient);
			leftFront.move_voltage(Utils::perToVol(spd) * -coefficient);
			pros::delay(20);

		}
		rightBack.move_voltage(0);
		rightFront.move_voltage(0);
		leftBack.move_voltage(0);
		leftFront.move_voltage(0);


	};

