#pragma once
#include "main.h"
#include "structDefs.hpp"
#include "api.h"

class OdomDisplay {
	private:
		lv_obj_t* odomScr = nullptr;
		lv_obj_t* visScr = nullptr;
		lv_obj_t*  field = nullptr;
		lv_obj_t* bot = nullptr;
		lv_obj_t* container = nullptr;
		lv_obj_t* statusLabel = nullptr;
		lv_obj_t* odomSwitch = nullptr;
		lv_obj_t* odomLabel = nullptr;

		lv_style_t* cStyle = nullptr;
		lv_obj_t* encLabel = nullptr;
		lv_obj_t* botLine = nullptr;
		lv_obj_t* visSwitch = nullptr;
		lv_obj_t* visLabel = nullptr;


		lv_obj_t* screen = nullptr;
		lv_obj_t* colorlabel = nullptr;
		lv_obj_t* mogo = nullptr;
		lv_style_t mStyle;
		lv_style_t fStyle;
		double fieldScreenDim;
		double robotRatio = 10;
		double fieldDim;
		lv_point_t botLinePoints[2];
		int lineThickness = 1.5;
		static constexpr double scalar = 1;
		pros::vision_object_s_t rtn;

	public:
		bool onVision = false;
		OdomDisplay(lv_obj_t* parent);
		OdomDisplay(lv_obj_t* parent, lv_color_t color);
		void setState(Point state, double theta);
		void encoderDebug(double encValue, std::string labelEnc);
		void update();
		void setVision(pros::vision_object_s_t inp);
};
