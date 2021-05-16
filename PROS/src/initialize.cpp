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




void initialize()
{
    /*lv_obj_t* bot = lv_led_create(lv_scr_act(), NULL);
    lv_led_on(bot);
    lv_style_t styleBot;
    lv_style_copy(&styleBot, &lv_style_plain);
    styleBot.body.border.color = LV_COLOR_WHITE;
    styleBot.body.main_color = LV_COLOR_RED;
    styleBot.body.grad_color = LV_COLOR_RED;
    styleBot.body.radius = LV_RADIUS_CIRCLE;
    lv_obj_set_pos(bot, 0, 0);
    lv_obj_set_size(bot, lv_obj_get_width(lv_scr_act()) / 2, lv_obj_get_height(lv_scr_act()) / 2);
    lv_led_set_style(bot,&styleBot);
    lv_led_on(bot);
    lv_obj_set_drag(bot, true);
    inert.reset();
    while (inert.is_calibrating()) {
        pros::delay(10);
    }*/
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