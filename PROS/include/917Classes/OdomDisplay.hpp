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
		lv_style_t fStyle;
		double fieldScreenDim;
		double fieldDim;
	public:
		OdomDisplay(lv_obj_t* parent);
		OdomDisplay(lv_obj_t* parent, lv_color_t color);
		void setState(Point state, double theta);
		void encoderDebug(int encValue, std::string labelEnc);
};
