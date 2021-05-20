#include "OdomDebug.hpp"

OdomDebug::OdomDebug(lv_obj_t* parent) 
    : OdomDebug(parent, LV_COLOR_RED) {
};

OdomDebug::OdomDebug(lv_obj_t* parent, lv_color_t color) {
    //screen container creation
    container = (lv_obj_create(parent, NULL));
    lv_obj_set_size(container, lv_obj_get_width(parent), lv_obj_get_height(parent));
    lv_obj_align(container, NULL, LV_ALIGN_CENTER, 0, 0);
    cStyle = lv_obj_get_style(container);
    cStyle->body.main_color = color;
    cStyle->body.grad_color = color;
    cStyle->body.border.width = 0;
    cStyle->body.radius = 0;
    
    //field creation
    field = lv_obj_create(container, NULL);
    lv_obj_set_size(field, lv_obj_get_height(container), lv_obj_get_height(container));
    lv_obj_align(field, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);
    double fieldScreenDim = lv_obj_get_height(field);

    //field styles
    lv_style_copy(&fStyle, cStyle);
    //lv_style_copy(fStyle, cStyle);
    fStyle.body.main_color = LV_COLOR_GRAY;
    fStyle.body.grad_color = LV_COLOR_GRAY;
    lv_obj_set_style(field, &fStyle);

    //lines for tiles
    lv_style_t lineStyle;
    lv_style_copy(&lineStyle, &lv_style_plain);
    lineStyle.line.width = 2;
    lineStyle.line.opa = LV_OPA_100;
    lineStyle.line.color = LV_COLOR_BLACK;

    lv_point_t line_points[] = { {fieldScreenDim / 5, 0}, {fieldScreenDim / 5, fieldScreenDim} };
    lv_obj_t* line1;
    line1 = lv_line_create(field, NULL);
    lv_line_set_points(line1, line_points, 2);
    lv_obj_set_pos(line1, 10, 0);
    lv_obj_set_style(line1, &lineStyle);
    for (int i = 0; i < 5; i++) {
        /*lv_point_t line_points[] = { {fieldScreenDim/5*i, 0}, {fieldScreenDim / 5 * i, fieldScreenDim}};
        lv_obj_t* line1;
        line1 = lv_line_create(field, NULL);
        lv_line_set_points(line1, line_points, 2);
        lv_obj_get_style->& lineStyle;
        //lv_obj_set_style(line1, &lineStyle);*/
    }
    
    //robot creation
    bot = lv_led_create(field, NULL);
    lv_led_on(bot);
    lv_obj_set_size(bot, fieldScreenDim/ 10, fieldScreenDim / 10);
    lv_obj_align(bot, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_style_t* lStyle = lv_obj_get_style(bot);
    lStyle->body.radius = LV_RADIUS_CIRCLE;
    lStyle->body.main_color = color;
    lStyle->body.grad_color = color;
    lStyle->body.border.color = LV_COLOR_WHITE;
    lStyle->body.border.width = 2;
    lStyle->body.border.opa = LV_OPA_100;
    lv_obj_set_style(bot, lStyle);
    
    //status text creation
    statusLabel = lv_label_create(container, NULL);

    lv_style_t* textStyle = lv_obj_get_style(statusLabel);
    textStyle->text.color = LV_COLOR_WHITE;
    textStyle->text.opa = LV_OPA_100;
    lv_obj_set_style(statusLabel, textStyle);
    lv_label_set_text(statusLabel, "No Odom Data Provided");
    lv_obj_align(statusLabel, container, LV_ALIGN_CENTER, -lv_obj_get_width(container) / 2 + (lv_obj_get_width(container) - fieldScreenDim) / 2, 0);

};
void OdomDebug::setState(Point state, double theta) {
    lv_obj_set_pos(bot, state.x, state.y);
    std::string text =
        "X_in: " + std::to_string(state.x) + "\n" +
        "Y_in: " + std::to_string(state.y) + "\n" +
        "Theta_deg: " + std::to_string(theta) + "\n";
    lv_label_set_text(statusLabel, text.c_str());
    lv_obj_align(statusLabel, container, LV_ALIGN_CENTER, -lv_obj_get_width(container) / 2 + (lv_obj_get_width(container) - fieldDimensions) / 2, 0);
};