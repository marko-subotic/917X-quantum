#include "917Classes\DriveTrainController.hpp"
#include <math.h>
	


	double DriveTrainController::distance2Points(Point first, Point second) {
		return sqrt(pow(first.x - second.x, 2) + pow(first.y - second.y, 2));
	};

