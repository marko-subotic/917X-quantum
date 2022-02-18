#include "DriveTrainController.hpp"
#include "DriveTrainState.hpp"
#include "Utils.hpp"
#include "Globals.hpp"
#include <math.h>



	void DriveTrainController::turnToPoint(DriveTrainState * state, Point target, double liftPos, int mogoState) {
		double pointAng = Utils::angleToPoint(Point(target.x-state->getPos().x, target.y-state->getPos().y));
		double targetAng = pointAng - state->getTheta();
		if (targetAng > M_PI) targetAng -= 2* M_PI;
		else if (targetAng < -M_PI) targetAng += 2* M_PI;
        lift.move_absolute(Utils::redMotConv(liftPos) * LIFT_RATIO, 100);
		while (std::abs(targetAng)> minErrorDegrees) {
            lift.move_absolute(Utils::redMotConv(liftPos) * LIFT_RATIO, 100);
			pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
			targetAng = pointAng - state->getTheta();
			double spd;
			if (targetAng > M_PI) targetAng -= 2* M_PI;
			else if (targetAng < -M_PI) targetAng += 2*M_PI;
			if (fabs(targetAng) < AngleUntilLinear[mogoState]) {
				spd = linSpd;
			}if (fabs(targetAng) > M_PI-AngleUntilDecelerate[mogoState]) {
				spd = oSpeed[mogoState];
			}
			else {
				//https://www.desmos.com/calculator/frano6ozhv
				spd = (oSpeed[mogoState] - linSpd) / 2 * (1 + cos(M_PI/(M_PI-AngleUntilDecelerate[mogoState])*(M_PI-AngleUntilDecelerate[mogoState]-fabs(targetAng)))) + linSpd;
			}
            if (spd < 8) {
               // break;
            }
			int coefficient = 1;
			if (targetAng < 0) {
				coefficient *= -1;
			}

           // printf("%f\n", spd);
            rightBack.move_velocity((spd)*coefficient);
            rightMid.move_velocity((spd) * coefficient);
            rightFront.move_velocity((spd) * coefficient);

            leftMid.move_velocity((spd) * -coefficient);
            leftBack.move_velocity((spd) * -coefficient);
            leftFront.move_velocity((spd) * -coefficient);
			pros::delay(20);

		}
        printf("exiting");
        rightBack.move_voltage(0);
        rightMid.move_voltage(0);
        rightFront.move_voltage(0);

        leftMid.move_voltage(0);
        leftBack.move_voltage(0);
        leftFront.move_voltage(0);
	};

	void DriveTrainController::driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState) {
        

        double finalSpeed = finalSpeedForward;
        double initialSpeed = initialSpeedForward;
        double pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
        double targetAng = pointAng - state->getTheta();
        lift.move_absolute(Utils::redMotConv(liftPos) * LIFT_RATIO, 100);

        if (targetAng > M_PI) targetAng -= M_PI;
        else if (targetAng < -M_PI) targetAng += M_PI;
        if (inSpd < 0) {
            //if (targetAng > 0) targetAng -= M_PI;
            //else targetAng += M_PI;
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
        double origSpeed = std::abs(spd);
        spd = initialSpeed;
        double kAccel = (origSpeed / (initialSpeed * DistanceUntilAccelerate)) - (1 / DistanceUntilAccelerate);
        double kDecel = (origSpeed - finalSpeed) / (origSpeed * DistanceUntilDecelerateInches);
        double leftSpeed = spd;
        double rightSpeed = spd;
        double highestSpd = 0;
        double kParabola = (finalSpeedForward - fabs(inSpd)) / (pow(DistanceUntilDecelerateInches, 2));


        while (fabs(error) >= MinErrorInches) {
            lift.move_absolute(Utils::redMotConv(liftPos) * LIFT_RATIO, 100);


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
                spd = origSpeed;
                highestSpd = spd;
            }

            pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
            targetAng = pointAng - state->getTheta();
            if (targetAng > M_PI) targetAng -= 2*M_PI;
            else if (targetAng < -M_PI) targetAng += 2*M_PI;
            if (inSpd < 0) {
                //if (targetAng > 0) targetAng -= M_PI;
                //else targetAng += M_PI;
            }
                

            //speed correction keeps robot pointed towards the point it wants to drive too
            //spd/ospeed makes the corrections get bigger the faster the bot goes, angle for max error
            //is a constant that needs to be tuned
            double speedCorrection = pow(spd / origSpeed, 2) * ((spd / AngleForMaxError[mogoState]) * targetAng);
            /*printf("correction: %f\n", speedCorrection);
            printf("(x,y,theta): (%f,%f,%f)\n", state->getPos().x, state->getPos().y, targetAng);
            // printf("%f\n", state->getPos().y);
            //printf("%f\n", error);
            printf("error: %f\n\n", (Utils::perToVol(leftSpeed)));
            */
            leftSpeed = spd - speedCorrection;
            rightSpeed = spd + speedCorrection;
            if (inSpd < 0) {
                //printf("speed: %f\n", spd);
                //printf("kParabola: %f\n", kParabola);
                double temp = leftSpeed;
                leftSpeed = -rightSpeed;
                rightSpeed = -temp;
                if (leftSpeed < -100) leftSpeed = -100;

                if (rightSpeed < -100) rightSpeed = -100;
            }
            else {
                if (leftSpeed > 100) leftSpeed = 100;

                if (rightSpeed > 100) rightSpeed = 100;
            }
            

            rightBack.move(Utils::perToVol(rightSpeed));
            rightMid.move(Utils::perToVol(rightSpeed));
            rightFront.move(Utils::perToVol(rightSpeed));

            leftMid.move(Utils::perToVol(leftSpeed));
            leftBack.move(Utils::perToVol(leftSpeed));
            leftFront.move(Utils::perToVol(leftSpeed));
            error = Utils::distanceBetweenPoints(target, state->getPos());
            rightBack.tare_position();
            rightFront.tare_position();
            rightMid.tare_position();

            leftMid.tare_position();
            leftBack.tare_position();
            leftFront.tare_position();
            pros::delay(20);
        }
        
        rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        leftMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        rightBack.move_absolute(0, 100);
        rightFront.move_absolute(0, 100);
        rightMid.move_absolute(0, 100);

        leftMid.move_absolute(0, 100);
        leftBack.move_absolute(0, 100);
        leftFront.move_absolute(0, 100);
	};