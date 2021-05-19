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


void initTask(void* p) {
    lv_obj_t* led = lv_led_create(lv_scr_act(), NULL);
    lv_obj_set_pos(led, 50, 50);
    lv_style_t* ledStyle = lv_obj_get_style(led);
    ledStyle->body.radius = LV_RADIUS_CIRCLE;
    ledStyle->body.main_color = LV_COLOR_RED;
    ledStyle->body.grad_color = LV_COLOR_RED;
    ledStyle->body.border.color = LV_COLOR_WHITE;
    ledStyle->body.border.width = 2;
    ledStyle->body.border.opa = LV_OPA_100;
    lv_obj_t* label = lv_label_create(lv_scr_act(), NULL);
    std::string text = CURRENT_TASK ? "not null" : "null";
    lv_label_set_text(label, text.c_str());
    pros::delay(1000);
    text = "calibrating";
    lv_label_set_text(label, text.c_str());
    //inert.reset();
    while(inert.is_calibrating()) {
        std::string text = "calibrating";
        lv_label_set_text(label, text.c_str());
       pros::delay(20);
    }
    ledStyle->body.main_color = LV_COLOR_NAVY;
    ledStyle->body.grad_color = LV_COLOR_NAVY;
    lv_led_on(led);
    lv_obj_invalidate(led);
    //lv_obj_del(led);
}

void initialize()
{   
    pros::Task initer(initTask);
    
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    
}

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