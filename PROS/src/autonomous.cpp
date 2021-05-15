#include "main.h"
#pragma once



void autonomous() {
    pros::delay(100);
    lv_obj_t* label = lv_label_create(lv_scr_act(), NULL);
    std::string text = "Enc Value: " + std::to_string(inert.get_heading());
    lv_label_set_text(label, text.c_str());
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, 0);
    while (1) {
        std::string text = "Enc Value: " + std::to_string(inert.get_heading());
        lv_label_set_text(label, text.c_str());
        pros::delay(100);
    }
    

}