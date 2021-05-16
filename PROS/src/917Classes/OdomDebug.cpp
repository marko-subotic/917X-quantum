#include "OdomDebug.hpp"

OdomDebug::OdomDebug(lv_obj_t* parent) {
    
    container = (lv_obj_create(parent, NULL));
    lv_obj_set_size(container, lv_obj_get_width(parent), lv_obj_get_height(parent));
    lv_obj_align(container, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_style_copy(&cStyle, &lv_style_plain_color);
    cStyle.body.main_color = LV_COLOR_GRAY;
    cStyle.body.grad_color = LV_COLOR_GRAY;
    cStyle.body.border.width = 0;
    cStyle.body.radius = 0;
    lv_obj_set_style(container, &cStyle);

    field = lv_obj_create(container, NULL);
    lv_style_copy(&fStyle, &cStyle);
    fStyle.body.main_color = LV_COLOR_RED;
    fStyle.body.grad_color = LV_COLOR_RED;
    lv_obj_set_style(field, &fStyle);
    //lv_obj_set_pos(field, lv_obj_get_width(lv_scr_act()) / 2, 0);
    lv_obj_set_size(field, lv_obj_get_height(container), lv_obj_get_height(container));
    lv_obj_align(field, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);
};

OdomDebug::OdomDebug(lv_obj_t* parent, lv_color_t color) {
    lv_obj_t* field = lv_obj_create(parent, NULL);
    lv_style_t styleField;
    lv_style_copy(&styleField, &lv_style_plain);
    styleField.body.border.color = LV_COLOR_BLACK;
    styleField.body.main_color = color;
    styleField.body.grad_color = color;
    styleField.body.radius = 0;
    styleField.body.border.width = 0;
    lv_obj_set_style(field, &styleField);
    //lv_obj_set_pos(field, lv_obj_get_width(lv_scr_act()) / 2, 0);
    lv_obj_set_size(field, lv_obj_get_height(parent), lv_obj_get_height(parent));
    lv_obj_align(field, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);
};
void OdomDebug::setState(Point state) {
	bot = lv_led_create(field, NULL);
};