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
		TEST_METHOD(TestCenterRotatePI6)
		{
			double dTheta = M_PI/6;
			Startup();
			Point centerRotate(0, user.distanceX / 2);
			//double leftEncRead = 
			double y = user.m_y;
			Assert::AreEqual(y, dTheta);
		}
	};
	double QuantumTests::minDif;
	DriveTrainState QuantumTests::user;

}
