#pragma once
#include "main.h"
#include "structDefs.hpp"
#include "api.h"

class OdomDisplay {
	private:
		lv_obj_t*  field = nullptr;
		lv_obj_t* bot = nullptr;
		lv_obj_t* container = nullptr;
		lv_obj_t* statusLabel = nullptr;
		lv_style_t* cStyle = nullptr;
		lv_obj_t* encLabel = nullptr;
		lv_obj_t* botLine = nullptr;
		lv_style_t fStyle;
		double fieldScreenDim;
		double robotRatio = 10;
		double fieldDim;
		lv_point_t botLinePoints[2];
		int lineThickness = 1.5;

	public:
		OdomDisplay(lv_obj_t* parent);
		OdomDisplay(lv_obj_t* parent, lv_color_t color);
		void setState(Point state, double theta);
		void encoderDebug(double encValue, std::string labelEnc);
};
