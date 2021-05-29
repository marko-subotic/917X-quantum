#pragma once
#include "main.h"
#include "structDefs.hpp"
#include "api.h"

class OdomDebug {
	private:
		lv_obj_t*  field = nullptr;
		lv_obj_t* bot = nullptr;
		lv_obj_t* container = nullptr;
		lv_obj_t* statusLabel = nullptr;
		lv_style_t* cStyle = nullptr;
		lv_style_t fStyle;
		double fieldScreenDim;
		double fieldDim;
	public:
		OdomDebug(lv_obj_t* parent);
		OdomDebug(lv_obj_t* parent, lv_color_t color);
		void setState(Point state, double theta);
};
