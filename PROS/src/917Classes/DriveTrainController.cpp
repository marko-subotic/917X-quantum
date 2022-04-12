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

        
        double firstAng = targetAng;
        lift.move_absolute(Utils::redMotConv(liftPos) * LIFT_RATIO, 100);


        double kParabola = (oSpeed[mogoState] - linSpd) / (pow((AngleWhenDecelerate[mogoState]-AngleUntilLinear[mogoState]), turnPow));
        double spd = 20;
        double prevLeft = leftEnc.get_position()/100;
        double prevRight = rightEnc.get_position()/100;

        while (std::abs(targetAng)> minErrorDegrees&&firstAng*targetAng>0) {
            lift.move_absolute(Utils::redMotConv(liftPos) * LIFT_RATIO, 100);
			pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
			targetAng = pointAng - state->getTheta();
            double targetSpd;
            
            double deltaTheta = (fabs(leftEnc.get_position()/100 - prevLeft) + fabs(rightEnc.get_position()/100 - prevRight)) / 2;
            //double encoderRPM = deltaTheta / (loopDelay / 1000) / 360;
            double aveRealVelo = deltaTheta / loopDelay * 1000 / 360 * smallDiam / bigDiam * 60 / rpms * 100;//(fabs(rightMid.get_actual_velocity()) + fabs(rightBack.get_actual_velocity()) + fabs(rightFront.get_actual_velocity()) + fabs(leftMid.get_actual_velocity()) + fabs(leftBack.get_actual_velocity()) + fabs(leftFront.get_actual_velocity())) / 6;
            prevLeft = leftEnc.get_position()/100, prevRight = rightEnc.get_position()/100;
            if (targetAng > M_PI) targetAng -= 2* M_PI;
			else if (targetAng < -M_PI) targetAng += 2*M_PI;


			if (fabs(targetAng) < AngleUntilLinear[mogoState]) {
				targetSpd = linSpd;
                
			}else if (fabs(targetAng-lookAhead) > AngleWhenDecelerate[mogoState]) {
				targetSpd = oSpeed[mogoState];
			}
			else {
				

                targetSpd = kParabola * (pow((fabs(targetAng-lookAhead) - AngleUntilLinear[mogoState]), turnPow)) + linSpd;
                
                
			}
            int coefficient = 1;
			if (targetAng < 0) {
				coefficient *= -1;
			}
            double compensation = (targetSpd - aveRealVelo) * kOsc[mogoState];
            spd += compensation;
            if (spd > 100) spd = 100;
            else if (spd < minCorrect[mogoState]) spd = -minCorrect[mogoState];
          
            //printf("speed: %f\n", spd);


            //printf("targetAng: %f, targetSpd: %f, realVelo: %f, spd: %f\n", targetAng, targetSpd, aveRealVelo, spd);

            rightBack.move(Utils::perToVol(spd * coefficient));
            rightMid.move(Utils::perToVol(spd * coefficient));
            rightFront.move(Utils::perToVol(spd * coefficient));

            leftMid.move(Utils::perToVol((spd) * -coefficient));
            leftBack.move(Utils::perToVol((spd) * -coefficient));
            leftFront.move(Utils::perToVol((spd) * -coefficient));
          
            rightBack.tare_position();
            rightFront.tare_position();
            rightMid.tare_position();

            leftMid.tare_position();
            leftBack.tare_position();
            leftFront.tare_position();
			pros::delay(loopDelay);

		}
        rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        leftMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        //printf("exiting");
        rightBack.move_absolute(0, 100);
        rightFront.move_absolute(0, 100);
        rightMid.move_absolute(0, 100);

        leftMid.move_absolute(0, 100);
        leftBack.move_absolute(0, 100);
        leftFront.move_absolute(0, 100);
	};

	void DriveTrainController::driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState) {
        

        double finalSpeed = finalSpeedForward[mogoState];
        double initialSpeed = initialSpeedForward[mogoState];
        double pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
        double targetAng = pointAng - state->getTheta();
        lift.move_absolute(Utils::redMotConv(liftPos) * LIFT_RATIO, 100);
        if (targetAng > M_PI) targetAng -= M_PI;
        else if (targetAng < -M_PI) targetAng += M_PI;
        printf("targetAng: %f\n", targetAng);
        if (inSpd < 0) {
            //if (targetAng > 0) targetAng -= M_PI;
            //else targetAng += M_PI;
            finalSpeed = finalSpeedBackward[mogoState];
            initialSpeed = initialSpeedBackward[mogoState];
        }
        const double NonMaxSpeedDist = DistanceUntilDecelerateInches[mogoState]+DistanceUntilAccelerate;
        double error = Utils::distanceBetweenPoints(target, state->getPos());
        //to give the bot time to slow over small distance
        if (fabs(error) < NonMaxSpeedDist) {
            initialSpeed=finalSpeed;
        }
        double dist = error;
        double origSpeed = std::abs(inSpd);
        double spd = initialSpeed;
        double prevLeft = leftEnc.get_position()/100;
        double prevRight = rightEnc.get_position()/100;
        double kAccel = (origSpeed / (initialSpeed * DistanceUntilAccelerate)) - (1 / DistanceUntilAccelerate);
        double leftSpeed = spd;
        double rightSpeed = spd;
        double highestSpd = 0;
        double kParabola = (fabs(inSpd) - finalSpeed) / (pow(DistanceUntilDecelerateInches[mogoState], decelPow));
        if (fabs(inSpd) < initialSpeed) {
            initialSpeed = fabs(inSpd);
        }
        //printf("%f")
        double distanceDecelerating = DistanceUntilDecelerateInches[mogoState] ;
        int crashCounter = 0;
        while (fabs(error) >= MinErrorInches) {
            lift.move_absolute(Utils::redMotConv(liftPos) * LIFT_RATIO, 100);
            double targetSpd;
            double deltaTheta = (fabs(leftEnc.get_position()/100 - prevLeft) + fabs(rightEnc.get_position()/100 - prevRight)) / 2;
            //double encoderRPM = deltaTheta / (loopDelay / 1000) / 360;
            double aveRealVelo = deltaTheta / loopDelay * 1000 / 360 * smallDiam / bigDiam * 60 / rpms * 100;//(fabs(rightMid.get_actual_velocity()) + fabs(rightBack.get_actual_velocity()) + fabs(rightFront.get_actual_velocity()) + fabs(leftMid.get_actual_velocity()) + fabs(leftBack.get_actual_velocity()) + fabs(leftFront.get_actual_velocity())) / 6;
            prevLeft = leftEnc.get_position()/100, prevRight = rightEnc.get_position()/100;
            bool isAccel = false;
            bool isDecel = false;
            double distanceCovered = dist-error;
            if (aveRealVelo < 1) {
                crashCounter++;
            }
            else {
                crashCounter = 0;
            }
            if (crashCounter > 10) {
                break;
            }
            if (fabs(dist) < NonMaxSpeedDist) {
                if (error > dist * (DistanceUntilDecelerateInches[mogoState] / (NonMaxSpeedDist)))
                {
                    distanceDecelerating = dist * (DistanceUntilDecelerateInches[mogoState]) / NonMaxSpeedDist;
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
                else if (fabs(error) <= DistanceUntilDecelerateInches[mogoState]) {
                    isDecel = true;
                }
            }
            if (isAccel) {
                targetSpd = initialSpeed * (1 + fabs(distanceCovered) * kAccel);
                highestSpd = targetSpd;
                kParabola = (fabs(highestSpd) - fabs(finalSpeed)) / (pow(distanceDecelerating, decelPow));

            }
            else if (isDecel) {
                //https://www.desmos.com/calculator/n5xuodzf4s
                targetSpd = kParabola * (pow(error, decelPow)) + finalSpeed;

            }
            else {
                targetSpd = origSpeed;
                highestSpd = targetSpd;
            }

            double compensation = (targetSpd - aveRealVelo) * kCor[mogoState];
            spd = targetSpd;
            if (spd > 100) spd = 100;
            else if (spd < -100) spd = -100;
            pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
            targetAng = pointAng - state->getTheta();
            if (targetAng > M_PI) targetAng -= 2*M_PI;
            else if (targetAng < -M_PI) targetAng += 2*M_PI;
            
                

            //speed correction keeps robot pointed towards the point it wants to drive too
            //spd/ospeed makes the corrections get bigger the faster the bot goes, angle for max error
            //is a constant that needs to be tuned
            double speedCorrection = pow(spd / origSpeed, 3)* (spd / AngleForMaxError[mogoState])* pow(targetAng, 1);
            /*printf("correction: %f\n", speedCorrection);
            printf("(x,y,theta): (%f,%f,%f)\n", state->getPos().x, state->getPos().y, targetAng);
            // printf("%f\n", state->getPos().y);
            //printf("%f\n", error);
            */
            //printf("%f, %f, %f\n", error, (spd), aveRealVelo);
            //printf("(x,y,theta): (%f,%f,%f)\n", state->getPos().x, state->getPos().y, targetAng);

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
            pros::delay(loopDelay);
        }
        printf("(x,y): (%f,%f)\n", state->getPos().x, state->getPos().y);

        rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        leftMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        /*rightBack.move_absolute(0, 30);
        rightFront.move_absolute(0, 30);
        rightMid.move_absolute(0, 30);

        leftMid.move_absolute(0, 30);
        leftBack.move_absolute(0, 30);
        leftFront.move_absolute(0, 30);
        */
        rightBack.move_velocity(0);
        rightFront.move_velocity(0);
        rightMid.move_velocity(0);

        leftMid.move_velocity(0);
        leftBack.move_velocity(0);
        leftFront.move_velocity(0);
        printf("(x,y): (%f,%f)\n", state->getPos().x, state->getPos().y);
	};