#include "main.h"

pros::MotorGroup left_motors_drivetrain({3, 12, 13});
pros::MotorGroup right_motors_drivetrain({8, -20, 18});
pros::Controller main_controller(pros::E_CONTROLLER_MASTER);

// init variables for joystick values
int LEFT_X_AXIS = main_controller.get_analog(
    pros::E_CONTROLLER_ANALOG_LEFT_X); // LEFT STICK X AXIS
int LEFT_Y_AXIS = main_controller.get_analog(
    pros::E_CONTROLLER_ANALOG_LEFT_Y); // LEFT STICK Y AXIS

int RIGHT_X_AXIS = main_controller.get_analog(
    pros::E_CONTROLLER_ANALOG_RIGHT_X); // RIGHT STICK X AXIS
int RIGHT_Y_AXIS = main_controller.get_analog(
    pros::E_CONTROLLER_ANALOG_RIGHT_Y); // RIGHT STICK Y AXIS

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  pros::lcd::register_btn1_cb(on_center_button);
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
void competition_initialize() {}

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
void autonomous() {}

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
void opcontrol() {
  while (true) {
    pros::lcd::print(0, "%d %d %d",
                     (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                     (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                     (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >>
                         0); // Prints status of the emulated screen LCDs

    // Arcade control scheme
	
    int dir = main_controller.get_analog(ANALOG_RIGHT_Y); // Gets amount forward/backward from right joystick
    int turn = main_controller.get_analog(ANALOG_LEFT_X); // Gets the turn left/right from left joystick

    left_motors_drivetrain.move(dir - turn);  // Sets left motor voltage
    right_motors_drivetrain.move(dir + turn); // Sets right motor voltage

    pros::delay(20);                          // Run for 20 ms then update
  }
}

int forward = -main_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
int turn = main_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
void arcade_drive(const int forward, const int turn) {
  left_motors_drivetrain.move(forward + turn);
  right_motors_drivetrain.move(forward - turn);
}