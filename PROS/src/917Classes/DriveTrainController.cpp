#include "DriveTrainController.hpp"
#include "DriveTrainState.hpp"
#include "Utils.hpp"
#include "Globals.hpp"
#include <math.h>



	void DriveTrainController::turnToPoint(DriveTrainState * state, Point target, double forkPos, double liftPos) {
		double pointAng = Utils::angleToPoint(Point(target.x-state->getPos().x, target.y-state->getPos().y));
		double targetAng = pointAng - state->getTheta();
		if (targetAng > M_PI) targetAng -= 2* M_PI;
		else if (targetAng < -M_PI) targetAng += 2* M_PI;
        bident.move_absolute(forkPos * FORK_RATIO, 100);
        lift.move_absolute(liftPos * LIFT_RATIO, 100);
		while (std::abs(targetAng)*100 > minErrorDegrees*100 ) {
            printf("%f", std::abs(targetAng) - minErrorDegrees);
            printf(", %f\n", minErrorDegrees);
			pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
			targetAng = pointAng - state->getTheta();
			double spd;
			if (targetAng > M_PI) targetAng -= 2* M_PI;
			else if (targetAng < -M_PI) targetAng += 2*M_PI;
			if (fabs(targetAng) < AngleUntilLinear) {
				spd = linSpd;
			}if (fabs(targetAng) > M_PI-AngleUntilDecelerate) {
				spd = oSpeed;
			}
			else {
				//https://www.desmos.com/calculator/frano6ozhv
				spd = (oSpeed - linSpd) / 2 * (1 + cos(M_PI/(M_PI-AngleUntilDecelerate)*(M_PI-AngleUntilDecelerate-fabs(targetAng)))) + linSpd;
			}
            if (spd < 8) {
               // break;
            }
			int coefficient = 1;
			if (targetAng < 0) {
				coefficient *= -1;
			}
			;

			/*rightBack.move_voltage(Utils::perToVol(spd) * coefficient);
            rightMid.move_voltage(Utils::perToVol(spd) * coefficient);
            rightFront.move_voltage(Utils::perToVol(spd) * coefficient);
            
            leftMid.move_voltage(Utils::perToVol(spd) * -coefficient);
			leftBack.move_voltage(Utils::perToVol(spd) * -coefficient);
			leftFront.move_voltage(Utils::perToVol(spd) * -coefficient);*/
            rightBack.move_velocity((spd) * coefficient);
            rightMid.move_velocity((spd) * coefficient);
            rightFront.move_velocity((spd) * coefficient);

            leftMid.move_velocity((spd) * -coefficient);
            leftBack.move_velocity((spd) * -coefficient);
            leftFront.move_velocity((spd) * -coefficient);
			pros::delay(1);

		}
        printf("exiting");
        rightBack.move_voltage(0);
        rightMid.move_voltage(0);
        rightFront.move_voltage(0);

        leftMid.move_voltage(0);
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
        //to give the bot time to slow over small distance
        if (fabs(error) < NonMaxSpeedDist) {
            initialSpeed=finalSpeed;
        }
        double dist = error;
        double volatile spd = inSpd;
        double oSpeed = std::abs(spd);
        spd = initialSpeed;
        double kAccel = (oSpeed / (initialSpeed * DistanceUntilAccelerate)) - (1 / DistanceUntilAccelerate);
        double kDecel = (oSpeed - finalSpeed) / (oSpeed * DistanceUntilDecelerateInches);
        double leftSpeed = spd;
        double rightSpeed = spd;
        double highestSpd = 0;
        double kParabola = (finalSpeedForward - fabs(inSpd)) / (pow(DistanceUntilDecelerateInches, 2));


        while (fabs(error) >= MinErrorInches) {
           // printf("%f\n", state->getPos().y);
            //printf("%f\n", error);

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
                printf("error: %f\n", (error));

                spd = kParabola * (pow(DistanceUntilDecelerateInches - fabs(error), 2)) + highestSpd;

            }
            else {
                spd = oSpeed;
                highestSpd = spd;
            }

            pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
            targetAng = pointAng - state->getTheta();
            if (targetAng > M_PI) targetAng -= 2*M_PI;
            else if (targetAng < -M_PI) targetAng += 2*M_PI;
            if (inSpd < 0) {
                if (targetAng > 0) targetAng -= M_PI;
                else targetAng += M_PI;
            }
                

            //speed correction keeps robot pointed towards the point it wants to drive too
            //spd/ospeed makes the corrections get bigger the faster the bot goes, angle for max error
            //is a constant that needs to be tuned
            double speedCorrection = 0;// spd / oSpeed * ((spd / AngleForMaxError) * targetAng);

            leftSpeed = spd + speedCorrection;
            rightSpeed = spd - speedCorrection;
            if (inSpd < 0) {
                printf("speed: %f\n", spd);
                printf("kParabola: %f\n", kParabola);
                double temp = leftSpeed;
                leftSpeed = -rightSpeed;
                rightSpeed = -temp;
            }

            rightBack.move_velocity((rightSpeed));
            rightMid.move_velocity((rightSpeed));
            rightFront.move_velocity((rightSpeed));

            leftMid.move_velocity((leftSpeed));
            leftBack.move_velocity((leftSpeed));
            leftFront.move_velocity((leftSpeed));
            error = Utils::distanceBetweenPoints(target, state->getPos());
            pros::delay(1);
        }
        
        rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        leftMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        rightBack.move(0);
        rightFront.move(-0);
        rightMid.move(0);

        leftMid.move(0);
        leftBack.move(0);
        leftFront.move(0);
	};