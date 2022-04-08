#define _USE_MATH_DEFINES
#include "pch.h"
#include "CppUnitTest.h"
#include "DriveTrainState.hpp"
#include "Utils.hpp"
#include "math.h"
#include <vector>
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace QuantumTest

{
	

	TEST_CLASS(QuantumTests)
	{
	public:
		static DriveTrainState user;
		static double minDif;
		static double encWC;
		static void Startup() {
			Point reseter(0, 0);
			user.setState(reseter, 0);
			minDif = .001;
			encWC = 360 / user.encWheelSize / M_PI;

		}
		
		static std::vector<double> thetaCalcHelper(Point centerR) {
			std::vector<double> rtrn(6, 0);
			rtrn[0] = cos(fabs(atan((centerR.y - user.leftEnc.y) / (centerR.x - user.leftEnc.x))));
			rtrn[1] = cos(fabs(atan((centerR.y - user.rightEnc.y) / (centerR.x - user.rightEnc.x))));
			rtrn[2] = sin(fabs(atan((centerR.y - user.bottomEnc.y) / (centerR.x - user.bottomEnc.x))));
			rtrn[3] = Utils::distanceBetweenPoints(centerR, user.leftEnc);
			rtrn[4] = Utils::distanceBetweenPoints(centerR, user.rightEnc);
			rtrn[5] = Utils::distanceBetweenPoints(centerR, user.bottomEnc);
			return rtrn;
		}
		
		//this tests the theta calculation from the DriveTrainState step function, the 3 places are
		//to the left of the bot, the middle and the right
			TEST_METHOD(thetaCalcLeft)
			{
				Startup();
				Point centerR(-1.5 * user.distanceYs, user.distanceX * 2);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				user.step(encWC*theta * helper[3] * helper[0], encWC*theta * helper[4] * helper[1], encWC*theta * helper[5] * helper[2]);
				double dif = fabs(user.getTheta() - theta);
				wchar_t printer = (wchar_t)dif;
				Assert::IsTrue(dif<minDif, &printer);
			}

			TEST_METHOD(thetaCalcRight)
			{
				Startup();
				Point centerR(1.5 * user.distanceYs, user.distanceX * 2);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				user.step(-encWC*theta * helper[3] * helper[0], -encWC*theta * helper[4] * helper[1], -encWC*theta * helper[5] * helper[2]);
				double dif = fabs(user.getTheta() - theta);
				wchar_t printer = (wchar_t)dif;
				Assert::IsTrue(dif<minDif, &printer);
			}

			TEST_METHOD(thetaCalcMid)
			{
				Startup();
				Point centerR(-.25* user.distanceYs, user.distanceX * 2);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				user.step(-encWC * theta * helper[3] * helper[0], encWC * theta * helper[4] * helper[1], encWC * theta * helper[5] * helper[2]);
				double dif = fabs(user.getTheta() - theta);
				wchar_t printer = (wchar_t)dif;
				Assert::IsTrue(dif < minDif, &printer);
			}

		//this group of tests is similar to theta, but instead tests if the final location is correct,
		//not theta
			TEST_METHOD(posCalcLeftUp)
			{
				Startup();
				Point centerR(-1.5 * user.distanceYs, user.distanceX * 2);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				Point finalP = Utils::rotateAroundPoint(centerR, user.calcPoint, theta);
				finalP.y -= user.calcPoint.y;
				finalP.x -= user.calcPoint.x;
				user.step(encWC * theta * helper[3] * helper[0], encWC * theta * helper[4] * helper[1], encWC * theta * helper[5] * helper[2]);
				double dif = fabs(user.getPos() - finalP);
				wchar_t printer = (wchar_t)dif;
				Assert::IsTrue(dif < minDif, &printer);
			}

			TEST_METHOD(posCalcLeftDown)
			{
				Startup();
				Point centerR(-1.5 * user.distanceYs, -user.distanceX * 2);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				Point finalP = Utils::rotateAroundPoint(centerR, user.calcPoint, theta);
				finalP.y -= user.calcPoint.y;
				finalP.x -= user.calcPoint.x;
				user.step(encWC * theta * helper[3] * helper[0], encWC * theta * helper[4] * helper[1], -encWC * theta * helper[5] * helper[2]);
				double dif = fabs(user.getPos() - finalP);
				wchar_t printer = (wchar_t)dif;
				Assert::IsTrue(dif < minDif, &printer);
			}


			TEST_METHOD(posCalcRightUp)
			{
				Startup();
				Point centerR(1.5 * user.distanceYs, user.distanceX * 2);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				Point finalP = Utils::rotateAroundPoint(centerR, user.calcPoint, theta);
				finalP.y -= user.calcPoint.y;
				finalP.x -= user.calcPoint.x;
				user.step(-encWC * theta * helper[3] * helper[0], -encWC * theta * helper[4] * helper[1], encWC * theta * helper[5] * helper[2]);
				double dif = fabs(user.getPos() - finalP);
				wchar_t printer = (wchar_t)dif;
				Assert::IsTrue(dif < minDif, &printer);
			}

			TEST_METHOD(posCalcRightDown)
			{
				Startup();
				Point centerR(1.5 * user.distanceYs, -user.distanceX * 2);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				Point finalP = Utils::rotateAroundPoint(centerR, user.calcPoint, theta);
				finalP.y -= user.calcPoint.y;
				finalP.x -= user.calcPoint.x;
				user.step(-encWC * theta * helper[3] * helper[0], -encWC * theta * helper[4] * helper[1], -encWC * theta * helper[5] * helper[2]);
				double dif = fabs(user.getPos() - finalP);
				wchar_t printer = (wchar_t)dif;
				Assert::IsTrue(dif < minDif, &printer);
			}

			TEST_METHOD(posCalcMidUp)
			{
				Startup();
				Point centerR(-.25 * user.distanceYs, user.distanceX * 2);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				Point finalP = Utils::rotateAroundPoint(centerR, user.calcPoint, theta);
				finalP.y -= user.calcPoint.y;
				finalP.x -= user.calcPoint.x;
				user.step(-encWC * theta * helper[3] * helper[0], encWC * theta * helper[4] * helper[1], encWC * theta * helper[5] * helper[2]);
				double dif = fabs(user.getPos() - finalP);
				wchar_t printer = (wchar_t)dif;
				Assert::IsTrue(dif < minDif, &printer);
			}
		
			TEST_METHOD(posCalcMidDown)
			{
				Startup();
				Point centerR(-.25 * user.distanceYs, -user.distanceX * 2);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				Point finalP = Utils::rotateAroundPoint(centerR, user.calcPoint, theta);
				finalP.y -= user.calcPoint.y;
				finalP.x -= user.calcPoint.x;
				user.step(-encWC * theta * helper[3] * helper[0], encWC * theta * helper[4] * helper[1], -encWC * theta * helper[5] * helper[2]);
				double dif = fabs(user.getPos() - finalP);
				wchar_t printer = (wchar_t)dif;
				Assert::IsTrue(dif < minDif, &printer);
			}
		//this group of test is for the angleToPoint function, it tests the angle to a point
		//in all 4 quadrants
			TEST_METHOD(angleToPointQ2) {
				Startup();
				bool result = false;
				Point target(-0.5, sqrt(3) / 2);
				double dif = fabs(Utils::angleToPoint(target) - M_PI / 6);
				wchar_t printer = (wchar_t)dif;
				if (dif < minDif) {
					Assert::IsTrue(true, L"");
				}
				else {
					Assert::IsTrue(false, &printer);
				}
			}

			TEST_METHOD(angleToPointQ3) {
				Startup();
				bool result = false;
				Point target(-0.5, sqrt(3) / -2);
				double dif = fabs(Utils::angleToPoint(target) - (5* M_PI / 6));
				wchar_t printer = (wchar_t)dif;
				if (dif < minDif) {
					Assert::IsTrue(true, L"");
				}
				else {
					Assert::IsTrue(false, &printer);
				}
			}


			TEST_METHOD(angleToPointQ4) {
				Startup();
				bool result = false;
				Point target(0.5, sqrt(3) / -2);
				double dif = fabs(Utils::angleToPoint(target) - (7 * M_PI / 6));
				wchar_t printer = (wchar_t)dif;
				if (dif < minDif) {
					Assert::IsTrue(true, L"");
				}
				else {
					Assert::IsTrue(false, &printer);
				}
			}


			TEST_METHOD(angleToPointQ1) {
				Startup();
				bool result = false;
				Point target(0.5, sqrt(3) / 2);
				double dif = fabs(Utils::angleToPoint(target) - (11* M_PI / 6));
				wchar_t printer = (wchar_t)dif;
				if (dif < minDif) {
					Assert::IsTrue(true, L"");
				}
				else {
					Assert::IsTrue(false, &printer);
				}
			}
	};
	DriveTrainState QuantumTests::user;
	double QuantumTests::minDif;
	double QuantumTests::encWC;

}
