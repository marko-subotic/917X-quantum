#include "OdomDisplay.hpp"
#include "Utils.hpp"

OdomDisplay::OdomDisplay(lv_obj_t* parent) 
    : OdomDisplay(parent, LV_COLOR_RED) {
};

OdomDisplay::OdomDisplay(lv_obj_t* parent, lv_color_t color) {
    //screen container creation
    odomScr = lv_obj_create(NULL, NULL);
    lv_obj_clean(odomScr);
    container = (lv_obj_create(odomScr, NULL));
    lv_obj_set_size(container, lv_obj_get_width(odomScr), lv_obj_get_height(odomScr));
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
    fieldScreenDim = lv_obj_get_height(field);
    //field styles
    lv_style_copy(&fStyle, cStyle);
    //lv_style_copy(fStyle, cStyle);
    fStyle.body.main_color = LV_COLOR_GRAY;
    fStyle.body.grad_color = LV_COLOR_GRAY;
    lv_obj_set_style(field, &fStyle);

    static lv_style_t lineStyle;
    lv_style_copy(&lineStyle, &lv_style_plain);
    lineStyle.line.width = 3;
    lineStyle.line.color = LV_COLOR_BLACK;

    const int tileCount = 6;
    lv_point_t startPoint = { 0,0 };
    std::vector<lv_point_t> pointVector(2, startPoint);
    static std::vector<std::vector<lv_point_t>> vertLinePoints(tileCount-1, pointVector);
    static std::vector<std::vector<lv_point_t>> horLinePoints(tileCount - 1, pointVector);

    for (int i = 1; i < tileCount; i++) {
        vertLinePoints[i - 1][0].x = (int16_t)(fieldScreenDim / tileCount * i);
        vertLinePoints[i - 1][0].y = 0;
        vertLinePoints[i - 1][1].x = (int16_t)(fieldScreenDim / tileCount * i);
        vertLinePoints[i - 1][1].y = (int16_t)(fieldScreenDim);
        lv_obj_t* line = lv_line_create(field, NULL);
        lv_line_set_points(line, vertLinePoints[i-1].data(), vertLinePoints[i-1].size());
        lv_obj_set_free_num(line, i);

        lv_obj_set_style(line, &lineStyle);
    }
    for (int i = 1; i < tileCount; i++) {
        horLinePoints[i - 1][0].x = 0;
        horLinePoints[i - 1][0].y = (int16_t)(fieldScreenDim / tileCount * i);
        horLinePoints[i - 1][1].x = (int16_t)(fieldScreenDim);
        horLinePoints[i - 1][1].y = (int16_t)(fieldScreenDim / tileCount * i);
        lv_obj_t* line = lv_line_create(field, NULL);
        lv_line_set_points(line, horLinePoints[i - 1].data(), horLinePoints[i - 1].size());
        lv_obj_set_free_num(line, i);

        lv_obj_set_style(line, &lineStyle);
    }
    
    //robot creation
   
        //first is led creation

        bot = lv_led_create(field, NULL);
        lv_led_on(bot);
        lv_obj_set_size(bot, fieldScreenDim/ robotRatio, fieldScreenDim / robotRatio);
        lv_obj_align(bot, NULL, LV_ALIGN_CENTER, 0, 0);

        lv_style_t* lStyle = lv_obj_get_style(bot);
        lStyle->body.radius = LV_RADIUS_CIRCLE;
        lStyle->body.main_color = color;
        lStyle->body.grad_color = color;
        lStyle->body.border.color = LV_COLOR_WHITE;
        lStyle->body.border.width = 2;
        lStyle->body.border.opa = LV_OPA_100;
        lv_obj_set_style(bot, lStyle);
    
        //next is line creation
        botLine = lv_line_create(bot, NULL);
        lv_obj_set_style(botLine, &lineStyle);
        startPoint.x = lv_obj_get_width(bot)/2;
        botLinePoints[0] = startPoint, botLinePoints[1] = startPoint;
        botLinePoints[1].y = lv_obj_get_height(bot)/2;
        lv_line_set_points(botLine, botLinePoints, 2);

    
    //status text creation
    statusLabel = lv_label_create(container, NULL);

    lv_style_t* textStyle = lv_obj_get_style(statusLabel);
    textStyle->text.color = LV_COLOR_WHITE;
    textStyle->text.opa = LV_OPA_100;
    lv_obj_set_style(statusLabel, textStyle);
    lv_label_set_text(statusLabel, "No Odom Data Provided");
    lv_obj_align(statusLabel, container, LV_ALIGN_CENTER, -lv_obj_get_width(container) / 2 + (lv_obj_get_width(container) - fieldScreenDim) / 2, 0);

    //encoder direction checker label creation
    encLabel = lv_label_create(container, NULL);

    
    lv_obj_set_style(encLabel, textStyle);
    lv_obj_align(encLabel, container, LV_ALIGN_CENTER, -lv_obj_get_width(container) / 2 + (lv_obj_get_width(container) - fieldScreenDim) / 2, -lv_obj_get_height(container)*3/4);
    lv_label_set_text(encLabel, "");

    //odom switch button creation
    odomSwitch = lv_btn_create(container, NULL);
    lv_obj_set_size(odomSwitch, lv_obj_get_width(container) / 9, lv_obj_get_height(container) / 9);
    lv_obj_align(odomSwitch, container, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    lv_style_t btnStyle;
    lv_style_copy(&btnStyle, cStyle);
    btnStyle.body.main_color = LV_COLOR_BLACK;
    btnStyle.body.grad_color = LV_COLOR_BLACK;
    btnStyle.body.radius = 0;
    lv_obj_set_style(odomSwitch, &btnStyle);
    
    odomLabel = lv_label_create(odomSwitch, NULL);
    lv_obj_set_style(odomLabel, textStyle);
    lv_label_set_text(odomLabel, "vision");
    
    


    //vision display creation
    visScr = lv_obj_create(NULL, NULL);
    lv_obj_clean(visScr);
    //lv_obj_set_size(visScr, lv_obj_get_width(visScr), lv_obj_get_height(visScr));
    //lv_obj_align(visScr, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_style_t* vStyle = lv_obj_get_style(visScr);
    vStyle->body.main_color = LV_COLOR_BLACK;
    vStyle->body.grad_color = LV_COLOR_BLACK;
    vStyle->body.border.color = LV_COLOR_RED;
    vStyle->body.border.width = 2;
    vStyle->body.radius = 0;

    //visual switch button creation
    visSwitch = lv_btn_create(visScr, NULL);
    lv_obj_set_size(visSwitch, lv_obj_get_width(visScr) / 9, lv_obj_get_height(visScr) / 9);
    lv_obj_align(visSwitch, visScr, LV_ALIGN_IN_TOP_RIGHT, 0, 0);
    lv_obj_set_style(visSwitch, &btnStyle);

    visLabel = lv_label_create(visSwitch, NULL);
    lv_obj_set_style(visLabel, textStyle);
    lv_label_set_text(visLabel, "odom");
    //*/

   

    screen = lv_obj_create(visScr, NULL);
    lv_obj_set_size(screen, (int16_t)(VISION_FOV_WIDTH* scalar), (int16_t)(VISION_FOV_HEIGHT* scalar));
    lv_obj_align(screen, visScr, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    lv_obj_set_style(screen, vStyle);
    
    colorlabel = lv_label_create(visScr, NULL);
    lv_obj_align(colorlabel, visScr, LV_ALIGN_CENTER, 115, 0);// (int16_t)(lv_obj_get_width(screen) + (lv_obj_get_width(container) - lv_obj_get_width(screen)) / 2), 0);

    textStyle = lv_obj_get_style(colorlabel);
    textStyle->text.color = LV_COLOR_WHITE;
    textStyle->text.opa = LV_OPA_100;
    lv_label_set_text(colorlabel, "No Vision\nData Provided");

    vision.set_signature(RED_ID, &RED_SIG);
    vision.set_signature(BLUE_ID, &BLUE_SIG);
    vision.set_signature(YELLOW_ID, &YELLOW_SIG);
    //vision.set_signature(5, &DARK_YELLOW);
    //vision.set_signature(6, &LIGHT_YELLOW);
    vision.set_auto_white_balance(1);
    rtn = vision.get_by_size(0);

    mogo = lv_obj_create(screen, NULL);
    lv_obj_set_size(mogo, (int16_t)(rtn.width* scalar), (int16_t)(rtn.height* scalar));
    lv_obj_set_pos(mogo, (int16_t)(rtn.left_coord* scalar), (int16_t)(rtn.top_coord* scalar));
    
    lv_style_copy(&mStyle, cStyle);
    mStyle.body.main_color = LV_COLOR_RED;
    mStyle.body.grad_color = LV_COLOR_RED;
    lv_obj_set_style(mogo, &mStyle);

    /**/
    lv_scr_load(visScr);
    lv_scr_load(odomScr);


};


void OdomDisplay::setState(Point state, double theta) {
    lv_obj_set_pos(bot, (fieldScreenDim/FIELD_DIMENSIONS)*state.x-lv_obj_get_width(bot)/2,fieldScreenDim-((fieldScreenDim / FIELD_DIMENSIONS)* state.y)-lv_obj_get_height(bot) / 2);
    lv_obj_invalidate(bot);
    std::vector<Point> a(2,Point(0,0));
    a[0].x = lv_obj_get_width(bot) / 2, a[1].y = 0;
    a[1].x = botLinePoints[1].x, a[1].y = botLinePoints[1].y;
    Point newLinePoint = Utils::rotateAroundPoint(a[1], a[0], -theta);
    botLinePoints[0].x = newLinePoint.x, botLinePoints[0].y = newLinePoint.y;
    lv_line_set_points(botLine, botLinePoints, 2);
    lv_obj_invalidate(botLine);
    std::string text =
        "X_in: " + std::to_string(state.x) + "\n" +
        "Y_in: " + std::to_string(state.y) + "\n" +
        "Theta_deg: " + std::to_string(theta) + "\n";
    lv_label_set_text(statusLabel, text.c_str());
    lv_obj_align(statusLabel, container, LV_ALIGN_CENTER, -lv_obj_get_width(container) / 2 + (lv_obj_get_width(container) - FIELD_DIMENSIONS) / 3, 0);
};

void OdomDisplay::encoderDebug(double encValue, std::string encSpec) {
    std::string text = encSpec + ": " + std::to_string(encValue);
    lv_label_set_text(encLabel, text.c_str());
    lv_obj_align(encLabel, container, LV_ALIGN_CENTER, -lv_obj_get_width(container) / 2 + (lv_obj_get_width(container) - fieldScreenDim) / 2, -lv_obj_get_height(container) / 4);
};

void OdomDisplay::update() {
    if (onVision) {
        if (lv_btn_get_state(visSwitch) == LV_BTN_STATE_REL) {
            onVision = false;
            lv_scr_load(odomScr);
        }
    }
    else {
        if (lv_btn_get_state(odomSwitch) == LV_BTN_STATE_REL) {
            onVision = true;
            lv_scr_load(visScr);
        }
    }
}

void OdomDisplay::setVision(pros::vision_object_s_t inp) {
    rtn = inp;
    //printf("%d\n", rtn.signature);
    ///*
    std::string labelText = "(x,y): (" + std::to_string(inp.left_coord) + "," + std::to_string(inp.top_coord) + ")\n" +
        "(w,h): (" + std::to_string(inp.width) + "," + std::to_string(inp.height) + ")\n";
    if (rtn.type == pros::vision_object_type_e_t::E_VISION_OBJECT_COLOR_CODE) {
        mStyle.body.main_color = LV_COLOR_RED;
        mStyle.body.grad_color = LV_COLOR_BLUE;
        labelText += std::to_string(rtn.signature);
    }

    else if (rtn.width < 10) {
        labelText += "No Vision\nData Provided";
    }
    else if (rtn.signature == RED_ID) {
        mStyle.body.main_color = LV_COLOR_RED;
        mStyle.body.grad_color = LV_COLOR_RED;
        labelText += "mogo is red";


    }
    else if (rtn.signature == BLUE_ID) {
        mStyle.body.main_color = LV_COLOR_BLUE;
        mStyle.body.grad_color = LV_COLOR_BLUE;
        labelText += "mogo is blue";

    }
    else if (rtn.signature == YELLOW_ID) {
        mStyle.body.main_color = LV_COLOR_YELLOW;
        mStyle.body.grad_color = LV_COLOR_YELLOW;
        labelText += "mogo is yellow";


    }
    lv_label_set_text(colorlabel, labelText.c_str());

    lv_obj_set_style(mogo, &mStyle);
    lv_obj_set_size(mogo, (int16_t)(rtn.width * scalar), (int16_t)(rtn.height * scalar));
    lv_obj_set_pos(mogo, (int16_t)(rtn.left_coord * scalar), (int16_t)(rtn.top_coord * scalar));
    lv_obj_invalidate(mogo);
}