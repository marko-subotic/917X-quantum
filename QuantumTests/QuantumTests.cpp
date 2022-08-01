#define _USE_MATH_DEFINES
#include "pch.h"
#include "CppUnitTest.h"
#include "DriveTrainState.hpp"
#include "DriveTrainController.hpp"
#include "structDefs.hpp"
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
			rtrn[0] = cos(fabs(atan((centerR.y - user.leftEncP.y) / (centerR.x - user.leftEncP.x))));
			rtrn[1] = cos(fabs(atan((centerR.y - user.rightEncP.y) / (centerR.x - user.rightEncP.x))));
			rtrn[2] = sin(fabs(atan((centerR.y - user.bottomEncP.y) / (centerR.x - user.bottomEncP.x))));
			rtrn[3] = Utils::distanceBetweenPoints(centerR, user.leftEncP);
			rtrn[4] = Utils::distanceBetweenPoints(centerR, user.rightEncP);
			rtrn[5] = Utils::distanceBetweenPoints(centerR, user.bottomEncP);
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
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}

			TEST_METHOD(thetaCalcRight)
			{
				Startup();
				Point centerR(1.5 * user.distanceYs, user.distanceX * 2);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				user.step(-encWC*theta * helper[3] * helper[0], -encWC*theta * helper[4] * helper[1], -encWC*theta * helper[5] * helper[2]);
				double dif = fabs(user.getTheta() - theta);
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}

			TEST_METHOD(thetaCalcMid)
			{
				Startup();
				Point centerR(-.25* user.distanceYs, user.distanceX * 2);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				user.step(-encWC * theta * helper[3] * helper[0], encWC * theta * helper[4] * helper[1], encWC * theta * helper[5] * helper[2]);
				double dif = fabs(user.getTheta() - theta);
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}

		//this group of tests is similar to theta, but instead tests if the final location is correct,
		//not theta
			TEST_METHOD(posCalcLeftUp)
			{
				Startup();
				Point centerR(-1.5 * user.distanceYs, 2*user.distanceX);
				std::vector<double> helper = thetaCalcHelper(centerR);
				double theta = M_PI / 4;
				Point finalP = Utils::rotateAroundPoint(centerR, user.calcPoint, theta);
				finalP.y -= user.calcPoint.y;
				finalP.x -= user.calcPoint.x;
				user.step(encWC * theta * helper[3] * helper[0], encWC * theta * helper[4] * helper[1], encWC * theta * helper[5] * helper[2]);
				double dif = fabs(Utils::distanceBetweenPoints(user.getPos(), finalP));
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
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
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
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
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
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
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
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
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
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
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}
		//this group of test is for the angleToPoint function, it tests the angle to a point
		//in all 4 quadrants
			TEST_METHOD(angleToPointQ2) {
				Startup();
				bool result = false;
				Point target(-0.5, sqrt(3) / 2);
				double dif = fabs(Utils::angleToPoint(target) - M_PI / 6);
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}

			TEST_METHOD(angleToPointQ3) {
				Startup();
				bool result = false;
				Point target(-0.5, sqrt(3) / -2);
				double dif = fabs(Utils::angleToPoint(target) - (5* M_PI / 6));
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}


			TEST_METHOD(angleToPointQ4) {
				Startup();
				bool result = false;
				Point target(0.5, sqrt(3) / -2);
				double dif = fabs(Utils::angleToPoint(target) - (7 * M_PI / 6));
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}


			TEST_METHOD(angleToPointQ1) {
				Startup();
				bool result = false;
				Point target(0.5, sqrt(3) / 2);
				double dif = fabs(Utils::angleToPoint(target) - (11* M_PI / 6));
				
				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}
			TEST_METHOD(guidePoint1) {
				Startup();
				Point target(1, 1);
				Point start(0, 0);
				double theta = M_PI / 4;
				double dif = .5 - Utils::pointAligner(start, target, theta,2).x;

				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}

			TEST_METHOD(mogoReseterUp) {
				Startup();
				Point target(0, Utils::mogoRad);
				Point start(0, 0);
				double theta = 0;
				double dif = Utils::distanceBetweenPoints(target, Utils::mogoReset(start, theta));

				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}

			TEST_METHOD(mogoReseterRight) {
				Startup();
				Point target(Utils::mogoRad, 0);
				Point start(0, 0);
				double theta = 3*M_PI/2;
				double dif = Utils::distanceBetweenPoints(target, Utils::mogoReset(start, theta));

				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}

			TEST_METHOD(mogoReseterLeft) {
				Startup();
				Point target(-Utils::mogoRad,0);
				Point start(0, 0);
				double theta = M_PI/2;
				double dif = Utils::distanceBetweenPoints(target, Utils::mogoReset(start, theta));

				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}

			TEST_METHOD(mogoReseterDown) {
				Startup();
				Point target(0, -Utils::mogoRad);
				Point start(0, 0);
				double theta = M_PI;
				double dif = Utils::distanceBetweenPoints(target, Utils::mogoReset(start, theta));

				std::wstring printer = std::to_wstring(dif);
				Assert::IsTrue(dif < minDif, printer.c_str());
			}
	};
	DriveTrainState QuantumTests::user;
	double QuantumTests::minDif;
	double QuantumTests::encWC;

}
