#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include <iostream>

using namespace std;
// init drivetrain motor groups and controller
pros::MotorGroup left_motors_drivetrain({-4, -14, 13});
pros::MotorGroup right_motors_drivetrain({8, -20, 18});
pros::Controller main_controller(pros::E_CONTROLLER_MASTER);

// intake and transit motors
pros::MotorGroup
    upper_transit_motor({9, false}); // intake motor (intake and outtake)
pros::MotorGroup
    intake_transit_motor({-2, false}); // intake motor (intake and outtake)

// pneumatics
bool funnel_engaged = false;
pros::adi::Pneumatics funnel_pneumatic_right('A', false);
pros::adi::Pneumatics funnel_pneumatic_left('B', false);

// descore pneumatic
bool descore_pneumatic_state = false;
pros::adi::Pneumatics descore_pneumatic('C', true);

enum ball_conveyor_state {
  UPPER_GOAL, // intake balls || spin both up
  LOWER_GOAL, // outtake balls || spin INTAKE_TRANSIST up, UPPER_TRANSIT down
  OUTTAKE,    // outtake balls || spin both down
  STOPPED     // stop both motors
};
ball_conveyor_state current_ball_conveyor_state = STOPPED; // initial state

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

// bool
void checkControllerButtonPress() {
  if (pros::E_CONTROLLER_DIGITAL_L1) {
  }
  if (pros::E_CONTROLLER_DIGITAL_L2) {
    cout << "L2 is pressed\n";
  }
  if (pros::E_CONTROLLER_DIGITAL_R1) {
    cout << "R1 is pressed\n";
  }
  if (pros::E_CONTROLLER_DIGITAL_R2) {
    cout << "R2 is pressed\n";
  }
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

    checkControllerButtonPress(); // Check if any controller buttons are pressed
                                  // for testing

    // Arcade control scheme

    // init variables for joystick values
    int LEFT_X_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_LEFT_X); // LEFT STICK X AXIS

    int LEFT_Y_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_LEFT_Y); // LEFT STICK Y AXIS

    int RIGHT_X_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_RIGHT_X); // RIGHT STICK X AXIS

    int RIGHT_Y_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_RIGHT_Y); // RIGHT STICK Y AXIS

    left_motors_drivetrain.move(LEFT_Y_AXIS + RIGHT_X_AXIS);
    right_motors_drivetrain.move(LEFT_Y_AXIS - RIGHT_X_AXIS);
    // left_motors_drivetrain.move(LEFT_Y_AXIS);  // Sets left motor voltage
    // right_motors_drivetrain.move(LEFT_Y_AXIS); // Sets right motor voltage
    pros::lcd::print(0, "%d %d %d", LEFT_X_AXIS, LEFT_Y_AXIS, RIGHT_X_AXIS);
    // right_motors_drivetrain.move(power_arcade_drive -
    //                              turn_arcade_drive); // Sets right motor
    //                              voltage

    pros::delay(20); // Run for 20 ms then update
  }
}
