#include "DriveTrainController.hpp"
#include "DriveTrainState.hpp"
#include "Utils.hpp"
#include "Globals.hpp"
#include <math.h>



	void DriveTrainController::turnToPoint(DriveTrainState * state, Point target, double forkPos, double liftPos) {
		double pointAng = Utils::angleToPoint(Point(target.x-state->getPos().x, target.y-state->getPos().y));
		double targetAng = pointAng - state->getTheta();
		if (targetAng > M_PI) targetAng -= M_PI;
		else if (targetAng < -M_PI) targetAng += M_PI;
        bident.move_absolute(forkPos * FORK_RATIO, 100);
        lift.move_absolute(liftPos * LIFT_RATIO, 100);
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

	void DriveTrainController::driveToPoint(DriveTrainState* state, Point target, double inSpd, double forkPos, double liftPos) {
        

        double finalSpeed = finalSpeedForward;
        double initialSpeed = initialSpeedForward;
        double pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
        double targetAng = pointAng - state->getTheta();
        if (targetAng > M_PI) targetAng -= M_PI;
        else if (targetAng < -M_PI) targetAng += M_PI;
        if (inSpd < 0) {
            if (targetAng > 0) targetAng -= M_PI;
            else targetAng += M_PI;
            finalSpeed = finalSpeedBackward;
            initialSpeed = initialSpeedBackward;
        }
        const double NonMaxSpeedDist = DistanceUntilDecelerateInches;
        double error = Utils::distanceBetweenPoints(target, state->getPos());
        double dist = error;
        //to give the bot time to slow over small distance
        if (fabs(error) < NonMaxSpeedDist) {
            initialSpeed=finalSpeed;
        }
        double volatile spd = inSpd;
        double oSpeed = spd;
        spd = initialSpeed;
        double kAccel = (oSpeed / (initialSpeed * DistanceUntilAccelerate)) - (1 / DistanceUntilAccelerate);
        double kDecel = (oSpeed - finalSpeed) / (oSpeed * DistanceUntilDecelerateInches);
        double leftSpeed = spd;
        double rightSpeed = spd;
        double highestSpd = 0;
        double kParabola = (finalSpeedForward - inSpd) / (pow(DistanceUntilDecelerateInches, 2));


        while (fabs(error) >= minErrorDegrees) {

            bool isAccel = false;
            bool isDecel = false;
            double distanceCovered = dist-error;
            if (fabs(dist) < NonMaxSpeedDist) {
                if (fabs(distanceCovered) < fabs(dist) * (DistanceUntilAccelerate / (NonMaxSpeedDist))) {
                    isAccel = true;
                }
                else {
                    isDecel = true;
                }
            }
            else {
                if (fabs(distanceCovered) <= DistanceUntilAccelerate) {
                    isAccel = true;
                }
                else if (fabs(error) <= DistanceUntilDecelerateInches) {
                    isDecel = true;
                }
            }
            if (isAccel) {
                spd = initialSpeed * (1 + fabs(distanceCovered) * kAccel);
                highestSpd = spd;
                kParabola = (fabs(finalSpeed) - fabs(spd)) / (pow(DistanceUntilDecelerateInches, 2));

            }
            else if (isDecel) {
                spd = kParabola * (pow(DistanceUntilDecelerateInches - fabs(error), 2)) + highestSpd;

            }
            else {
                spd = oSpeed;
                highestSpd = spd;
            }


            pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
            targetAng = pointAng - state->getTheta();
            if (targetAng > M_PI) targetAng -= M_PI;
            else if (targetAng < -M_PI) targetAng += M_PI;
            if (inSpd < 0) {
                if (targetAng > 0) targetAng -= M_PI;
                else targetAng += M_PI;
            }
                

            //speed correction keeps robot pointed towards the point it wants to drive too
            //spd/ospeed makes the corrections get bigger the faster the bot goes, angle for max error
            //is a constant that needs to be tuned
            double speedCorrection = spd / oSpeed * ((spd / AngleForMaxError) * targetAng);

            leftSpeed = spd - speedCorrection;
            rightSpeed = spd + speedCorrection;
            if (dist < 0) {
                double temp = leftSpeed;
                leftSpeed = -rightSpeed;
                rightSpeed = -temp;
            }

            rightBack.move_voltage(Utils::perToVol(spd));
            rightFront.move_voltage(Utils::perToVol(spd));
            leftBack.move_voltage(Utils::perToVol(spd));
            leftFront.move_voltage(Utils::perToVol(spd));
            pros::delay(20);
            error = Utils::distanceBetweenPoints(target, state->getPos());
        }
        rightBack.move_voltage(0);
        rightFront.move_voltage(0);
        leftBack.move_voltage(0);
        leftFront.move_voltage(0);
	};