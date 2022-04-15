#include "DriveTrainController.hpp"
#include "DriveTrainState.hpp"
#include "Utils.hpp"
#include "Globals.hpp"
#include <math.h>

Point DriveTrainController::pointAligner(Point state, Point target, double finalAng) {
    double xPerp, yPerp;
    Point alignPoint(0, 0);
    if (abs(finalAng) < .01 || abs(abs(finalAng) - M_PI) < .01) {
        xPerp = target.x;
        yPerp = state.y;
    }else if (abs(abs(finalAng)-M_PI/2.0) < .01 || abs(abs(finalAng) - 3*M_PI/2) < .01) {
        xPerp = state.x;
        yPerp = target.y;
    }
    else {
        xPerp = (state.y - target.y + 1/tan(finalAng) * target.x + state.x * tan(finalAng)) / (1/tan(finalAng) + tan(finalAng));
        yPerp = (-tan(finalAng))*(xPerp - state.x) + state.y;
        printf("%f, %f\n", xPerp, yPerp);

    }
    //if (state.x <= target.x) {
        alignPoint.x = xPerp + (target.x - xPerp) * kDist;
    //}
    //else {
    //    alignPoint.x = xPerp - (target.x - xPerp) * kDist;
    //}
    //if (state.y <= target.y) {
        alignPoint.y = yPerp + (target.y - yPerp) * kDist;
    /* }
    else {

    }*/
    return alignPoint;
    }

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
        double veloAng;
        double dif;
        while (std::abs(targetAng)> minErrorDegrees&&firstAng*targetAng>0) {
            lift.move_absolute(Utils::redMotConv(liftPos) * LIFT_RATIO, 100);
			pointAng = Utils::angleToPoint(Point(target.x - state->getPos().x, target.y - state->getPos().y));
			targetAng = pointAng - state->getTheta();
            double targetSpd;


            double corSpd;
            double deltaTheta = (fabs(leftEnc.get_position()/100 - prevLeft) + fabs(rightEnc.get_position()/100 - prevRight)) / 2;
            //double encoderRPM = deltaTheta / (loopDelay / 1000) / 360;
            double aveRealVelo = deltaTheta / loopDelay * 1000 / 360 * smallDiam / bigDiam * 60 / rpms * 100;//(fabs(rightMid.get_actual_velocity()) + fabs(rightBack.get_actual_velocity()) + fabs(rightFront.get_actual_velocity()) + fabs(leftMid.get_actual_velocity()) + fabs(leftBack.get_actual_velocity()) + fabs(leftFront.get_actual_velocity())) / 6;
            dif = targetSpd - aveRealVelo;

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
            veloAng = fabs(targetAng) - 2 / fabs(dif);
            if (veloAng < 0) {
                veloAng = 0;
            }

            if (fabs(veloAng) < AngleUntilLinear[mogoState]) {
                corSpd = linSpd;
            }
            else if (fabs(veloAng) > AngleWhenDecelerate[mogoState]) {
                corSpd = oSpeed[mogoState];
            }
            else {
                corSpd = kParabola * (pow((fabs(veloAng) - AngleUntilLinear[mogoState]), turnPow)) + linSpd;
            }
            int coefficient = 1;
			if (targetAng < 0) {
				coefficient *= -1;
			}
            double compensation = (corSpd - aveRealVelo) * kOsc[mogoState];
            spd += compensation;
            if (spd > 100) spd = 100;
            else if (spd < minCorrect[mogoState]) spd = -minCorrect[mogoState];
          
            //printf("speed: %f\n", spd);


            printf("veloAng: %f, targetSpd: %f, realVelo: %f, spd: %f\n", corSpd, targetSpd, aveRealVelo, spd);

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
            if (!inAuton) {
                return;
            }

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

	void DriveTrainController::driveToPoint(DriveTrainState* state, Point target, double inSpd, double liftPos, int mogoState, double finalAng) {
        

        double finalSpeed = finalSpeedForward[mogoState];
        double initialSpeed = initialSpeedForward[mogoState];
        finalAng *= (M_PI / 180.0);
        Point alignPoint = DriveTrainController::pointAligner(state->getPos(), target, finalAng);
        double pointAng = Utils::angleToPoint(Point(alignPoint.x - state->getPos().x, alignPoint.y - state->getPos().y)); 
        double targetAng = pointAng - state->getTheta();
        turnToPoint(state, alignPoint, liftPos, mogoState);
        lift.move_absolute(Utils::redMotConv(liftPos) * LIFT_RATIO, 100);
        if (targetAng > M_PI) targetAng -= M_PI;
        else if (targetAng < -M_PI) targetAng += M_PI;
        printf("targetAng: %f\n", finalAng);
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
            if (!inAuton) {
                return;
            }
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
            alignPoint = DriveTrainController::pointAligner(state->getPos(), target, finalAng);
            pointAng = Utils::angleToPoint(Point(alignPoint.x - state->getPos().x, alignPoint.y - state->getPos().y)); 
            targetAng = pointAng - state->getTheta();
            if (targetAng > M_PI) targetAng -= 2*M_PI;
            else if (targetAng < -M_PI) targetAng += 2*M_PI;
            
                

            //speed correction keeps robot pointed towards the point it wants to drive too
            //spd/ospeed makes the corrections get bigger the faster the bot goes, angle for max error
            //is a constant that needs to be tuned
            double speedCorrection = pow(error / dist, 1)* (spd / AngleForMaxError[mogoState])* pow(targetAng, 1);
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