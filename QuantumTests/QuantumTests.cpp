#define _USE_MATH_DEFINES
#include "pch.h"
#include "CppUnitTest.h"
#include "917Classes\DriveTrainState.hpp"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace QuantumTest

{
	

	TEST_CLASS(QuantumTests)
	{
	public:
		static DriveTrainState user;
		static double minDif;
		static void Startup() {
			Point reseter(0, 0);
			user.setState(reseter, 0);
			minDif = .001;
		}
		static double calcVertEncDist(double dTheta, double centerRotateX, double vertEncX) {
			return dTheta * (vertEncX - centerRotateX);
		}
		static double calcHorEncDist(double dTheta, double centerRotateY, double horEncY) {
			return dTheta * (centerRotateY - horEncY);
		}
		TEST_METHOD(TestCenterRotatePI6)
		{
			double dTheta = M_PI/6;
			Startup();
			Point centerOfRotation((user.rightEnc.x + user.leftEnc.x) / 2, (user.rightEnc.y + user.bottomEnc.y) / 2);
			Point calcPoint((user.rightEnc.x + user.leftEnc.x) / 2, user.rightEnc.y);
			double wheelConstRecip = 360/(user.encWheelSize * M_PI);
			double leftEncRead = calcVertEncDist(dTheta, centerOfRotation.x, user.leftEnc.x) *wheelConstRecip;
			double rightEncRead = calcVertEncDist(dTheta, centerOfRotation.x, user.rightEnc.x) * wheelConstRecip;
			double horEncRead = calcHorEncDist(dTheta, centerOfRotation.y, user.bottomEnc.y) * wheelConstRecip;
			user.step(leftEncRead, rightEncRead, horEncRead, -dTheta*(180/M_PI), 0);
			/*double endY = 1;
			double endX = 1;*/
			Point userEndP = user.getPos();
			Point endP = user.rotateAroundPoint(centerOfRotation,calcPoint, dTheta);
			endP.x = endP.x - calcPoint.x;
			endP.y = endP.y - calcPoint.y;
			if (fabs(endP.x-userEndP.x)<minDif) {
				if (fabs(endP.y - userEndP.y) < minDif) {
					Assert::IsTrue(true, L"Test TestCenterRotatePI6 passed");
				}
				else {
					Assert::IsTrue(false, L"Test TestCenterRotatePI6 failed (y incorrect)");
				}
			}
			else {
				Assert::IsTrue(false, L"Test TestCenterRotatePI6 failed (x incorrect)");
			}
		}
	};
	double QuantumTests::minDif;
	DriveTrainState QuantumTests::user;

}
