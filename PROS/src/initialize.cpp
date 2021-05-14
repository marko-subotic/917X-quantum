#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
static lv_res_t btnm_action(lv_obj_t* btnm, const char* txt);

std::string selectedAuton = "None";
const char* btnarr_map[] = { "Home Row", "10B Grind", "\n",
                               "Home Row: Surface", "Home Row: No Mid", "\n",
                               "Skills", "None", "" };
void initialize() {
    lv_obj_t* bot = lv_obj_create(lv_scr_act(), NULL);
    lv_style_t styleBot;
    lv_style_copy(&styleBot, &lv_style_plain_color);
    styleBot.body.main_color = LV_COLOR_RED;
    styleBot.body.grad_color = LV_COLOR_RED;
    lv_obj_set_pos(bot, 0, 0);
    lv_obj_set_size(bot, lv_obj_get_width(lv_scr_act()) / 2, lv_obj_get_height(lv_scr_act()) / 2);
    lv_obj_set_style(bot, &styleBot);
    lv_obj_set_drag(bot, true);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    pros::delay(100);
    lv_obj_t* label = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_text(label, "917M Autonomous Selector:");
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, 0);

    lv_obj_t* btnm1 = lv_btnm_create(lv_scr_act(), NULL);
    lv_btnm_set_map(btnm1, btnarr_map);
    lv_btnm_set_action(btnm1, btnm_action);
    lv_obj_set_size(btnm1, LV_HOR_RES, LV_VER_RES * 5 / 6);
    lv_obj_align(btnm1, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);
    lv_btnm_set_toggle(btnm1, true, 5);

    //Styles, yay!
    static lv_style_t style_bg;
    lv_style_copy(&style_bg, &lv_style_plain);
    style_bg.body.main_color = LV_COLOR_BLACK;
    style_bg.body.grad_color = LV_COLOR_BLACK;
    style_bg.body.padding.hor = 0;
    style_bg.body.padding.ver = 0;
    style_bg.body.padding.inner = 0;
    static lv_style_t style_btn_rel;
    static lv_style_t style_btn_pr;
    static lv_style_t style_btn_tgl_pr;
    lv_style_copy(&style_btn_rel, &lv_style_btn_rel);
    style_btn_rel.body.main_color = LV_COLOR_MAKE(0x30, 0x30, 0x30);
    style_btn_rel.body.grad_color = LV_COLOR_BLACK;
    style_btn_rel.body.border.color = LV_COLOR_SILVER;
    style_btn_rel.body.border.width = 1;
    style_btn_rel.body.border.opa = LV_OPA_50;
    style_btn_rel.body.radius = 0;
    lv_style_copy(&style_btn_pr, &style_btn_rel);
    style_btn_pr.body.main_color = LV_COLOR_MAKE(0x55, 0x96, 0xd8);
    style_btn_pr.body.grad_color = LV_COLOR_MAKE(0x37, 0x62, 0x90);
    style_btn_pr.text.color = LV_COLOR_MAKE(0xbb, 0xd5, 0xf1);
    lv_style_copy(&style_btn_tgl_pr, &style_btn_rel);
    style_btn_tgl_pr.body.main_color = LV_COLOR_YELLOW;
    style_btn_tgl_pr.body.grad_color = LV_COLOR_ORANGE;
    style_btn_tgl_pr.text.color = LV_COLOR_BLACK;
    lv_btnm_set_style(btnm1, LV_BTNM_STYLE_BG, &style_bg);
    lv_btnm_set_style(btnm1, LV_BTNM_STYLE_BTN_PR, &style_btn_pr);
    lv_btnm_set_style(btnm1, LV_BTNM_STYLE_BTN_TGL_REL, &style_btn_tgl_pr);
}

static lv_res_t btnm_action(lv_obj_t* btnm, const char* txt) {
    selectedAuton = txt;
    return LV_RES_OK;
}


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */