#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include <iostream>

#define TOGGLE_DESCORER E_CONTROLLER_DIGITAL_L2 // define toggle descorer button

#define SPIN_FOR_UPPER_GOAL                                                    \
  E_CONTROLLER_DIGITAL_R2 // define spin for upper goal button

#define SPIN_FOR_MIDDLE_GOAL                                                   \
  E_CONTROLLER_DIGITAL_R1 // define spin for middle goal button

#define TOGGLE_INTAKE_FUNNEL                                                   \
  E_CONTROLLER_DIGITAL_B // define toggle funnel button

#define CONTROLLER_L1 E_CONTROLLER_DIGITAL_L1 // define controller L1 button

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
pros::adi::Pneumatics funnel_pneumatic_right('H', false);
pros::adi::Pneumatics funnel_pneumatic_left('G', false);

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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");
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

// function to update pneumatics based on states
void updatePneumatics() {
  if (funnel_engaged) {
    // engage funnel pneumatics when true
    funnel_pneumatic_right.set_value(true);
    funnel_pneumatic_left.set_value(true);
  } else if (!funnel_engaged) {
    // disengage funnel pneumatics when false
    funnel_pneumatic_right.set_value(false);
    funnel_pneumatic_left.set_value(false);
  }

  if (descore_pneumatic_state) {
    descore_pneumatic.set_value(true);
    // engage descore pneumatics when true
  } else if (!descore_pneumatic_state) {
    // disengage descore pneumatics when false
    descore_pneumatic.set_value(false);
  }
}
// function to check if any controller buttons are pressed
void checkControllerButtonPress() {
  if (main_controller.get_digital(pros::TOGGLE_INTAKE_FUNNEL)) {
    funnel_engaged = !funnel_engaged;
    pros::lcd::print(2, "L1 is pressed\n");

  } else if (main_controller.get_digital(pros::TOGGLE_DESCORER)) {
    descore_pneumatic_state = !descore_pneumatic_state;
    pros::lcd::print(2, "L2 is pressed\n");

  } else if (main_controller.get_digital(pros::SPIN_FOR_MIDDLE_GOAL)) {
    pros::lcd::print(2, "R1 is pressed\n");
    current_ball_conveyor_state = LOWER_GOAL;
  } else if (main_controller.get_digital(pros::SPIN_FOR_UPPER_GOAL)) {
    pros::lcd::print(2, "R2 is pressed\n");
    current_ball_conveyor_state = UPPER_GOAL;
  } else {
    pros::lcd::print(2, "No buttons pressed\n");
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

    checkControllerButtonPress(); // Check if any controller buttons are pressed
    updatePneumatics();           // Update pneumatics based on bool/enum states

    // init variables for joystick values
    int LEFT_X_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_LEFT_X); // LEFT STICK X AXIS

    int LEFT_Y_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_LEFT_Y); // LEFT STICK Y AXIS

    int RIGHT_X_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_RIGHT_X); // RIGHT STICK X AXIS

    int RIGHT_Y_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_RIGHT_Y); // RIGHT STICK Y AXIS

    // if the turn value is not large unough, disregard and just use
    // Arcade control scheme
    // forward/backward
    // PROBLEM: when both sticks are pushed, the robot does not move diagonally

    double left_motor_voltage =
        LEFT_Y_AXIS + RIGHT_X_AXIS; // left motor voltage calculation
    double right_motor_voltage =
        LEFT_Y_AXIS - RIGHT_X_AXIS; // right motor voltage calculation

    // Constants
    constexpr int MOTOR_MAX = 127;

    // Overflow transfer correction
    double difference = 0.0;

    // left motor overflow cases
    if (left_motor_voltage > MOTOR_MAX) {
      // Left is too positive (forward too strong)
      difference = (left_motor_voltage - MOTOR_MAX) / 2.0;
      left_motor_voltage = MOTOR_MAX;
      right_motor_voltage -= difference; // reduce right to preserve ratio
    } else if (left_motor_voltage < -MOTOR_MAX) {
      // Left is too negative (reverse too strong)
      difference = (left_motor_voltage + MOTOR_MAX) / 2.0;
      left_motor_voltage = -MOTOR_MAX;
      right_motor_voltage -=
          difference; // subtract because left is underflowing
    }

    // right motor overflow cases
    if (right_motor_voltage > MOTOR_MAX) {
      // Right is too positive (forward too strong)
      difference = (right_motor_voltage - MOTOR_MAX) / 2.0;
      right_motor_voltage = MOTOR_MAX;
      left_motor_voltage -= difference;
    } else if (right_motor_voltage < -MOTOR_MAX) {
      // Right is too negative (reverse too strong)
      difference = (right_motor_voltage + MOTOR_MAX) / 2.0;
      right_motor_voltage = -MOTOR_MAX;
      left_motor_voltage -= difference;
    }

    // double check to make sure we are in range
    if (left_motor_voltage > MOTOR_MAX)
      left_motor_voltage = MOTOR_MAX;
    if (left_motor_voltage < -MOTOR_MAX)
      left_motor_voltage = -MOTOR_MAX;
    if (right_motor_voltage > MOTOR_MAX)
      right_motor_voltage = MOTOR_MAX;
    if (right_motor_voltage < -MOTOR_MAX)
      right_motor_voltage = -MOTOR_MAX;

    main_controller.print(
        0, 0, "LX: %d LY: %d RX: %d RY: %d", LEFT_X_AXIS, LEFT_Y_AXIS,
        RIGHT_X_AXIS,
        RIGHT_Y_AXIS); // print joystick values to controller screen for testing

    main_controller.print(1, 0, "LVOLT: %.2f RVOLT: %.2f", left_motor_voltage,
                          right_motor_voltage); // print motor voltages to
                                                // controller screen for testing
    left_motors_drivetrain.move_voltage(left_motor_voltage);
    right_motors_drivetrain.move_voltage(right_motor_voltage);

    pros::delay(20); // Run for 20 ms then update to keep cpu happy :)
  }
}