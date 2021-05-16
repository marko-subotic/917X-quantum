#include "main.h"

void autonomous() {
    pros::delay(100);
    OdomDebug display(lv_scr_act());
    /*lv_obj_t* field = lv_led_create(lv_scr_act(), NULL);
    lv_obj_t* bot = lv_led_create(field, NULL);
    lv_led_on(bot);
    lv_style_t styleBot;
    lv_style_copy(&styleBot, &lv_style_plain);
    styleBot.body.border.color = LV_COLOR_WHITE;
    styleBot.body.main_color = LV_COLOR_RED;
    styleBot.body.grad_color = LV_COLOR_RED;
    styleBot.body.radius = LV_RADIUS_CIRCLE;
    lv_obj_set_pos(bot, lv_obj_get_width(lv_scr_act())/2, 0);
    lv_obj_set_size(bot, lv_obj_get_width(lv_scr_act()) / 2, lv_obj_get_height(lv_scr_act()) / 2);
    lv_led_set_style(bot, &styleBot);
    lv_led_on(bot);
    lv_obj_set_drag(bot, true);*/
    lv_obj_t* label = lv_label_create(lv_scr_act(), NULL);
    std::string text = "Enc Value: " + std::to_string(inert.get_yaw());
    lv_label_set_text(label, text.c_str());
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, 0);
    
    while (1) {
        std::string text = "Enc Value: " + std::to_string(inert.get_yaw());
        lv_label_set_text(label, text.c_str());
        pros::delay(100);
    }
    

}