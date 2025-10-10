#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"

#define EXPONENT 1.9 // the exponential curve for the joystick inputs

// defines for controller buttons for readability
#define CONTROLLER_R1 E_CONTROLLER_DIGITAL_R1
#define CONTROLLER_R2 E_CONTROLLER_DIGITAL_R2
#define CONTROLLER_L1 E_CONTROLLER_DIGITAL_L1
#define CONTROLLER_L2 E_CONTROLLER_DIGITAL_L2
#define CONTROLLER_B E_CONTROLLER_DIGITAL_B

using namespace std;
// init drivetrain motor groups and controller
pros::MotorGroup left_motors_drivetrain({-6, -14, 13});
pros::MotorGroup right_motors_drivetrain({8, -20, 18});
pros::Controller main_controller(pros::E_CONTROLLER_MASTER);

// pneumatics
bool funnel_engaged = false;
pros::adi::Pneumatics funnel_pneumatic_right('H', false);
pros::adi::Pneumatics funnel_pneumatic_left('G', false);

// intake and transit motors
pros::Motor upper_transit_motor(7);
pros::Motor intake_transit_motor(-2);

// enum for ball conveyer belt
enum ball_conveyor_state {
  UPPER_GOAL,  // intake balls || spin both up
  MIDDLE_GOAL, // outtake balls || spin INTAKE_TRANSIST up, UPPER_TRANSIT down
  OUTTAKE,     // outtake balls || spin both down
  STOPPED      // stop both motors
};

ball_conveyor_state current_ball_conveyor_state;

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
  left_motors_drivetrain.set_brake_mode(pros::MotorBrake::brake);
  right_motors_drivetrain.set_brake_mode(pros::MotorBrake::brake);

  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");
  current_ball_conveyor_state = STOPPED; // initial state
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // current_ball_conveyor_state = STOPPED;
  // funnel_engaged = false;
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
}

double custom_clamp(double input, double MIN_VALUE, double MAX_VALUE) {
  if (input < MIN_VALUE)
    return MIN_VALUE;
  if (input > MAX_VALUE)
    return MAX_VALUE;
  return input;
}

void updateBallConveyorMotors() {
  switch (current_ball_conveyor_state) {
  case UPPER_GOAL:
    upper_transit_motor.move(127);  // spin upper transit motor
    intake_transit_motor.move(127); // spin intake transit motor
    break;
  case MIDDLE_GOAL:
    upper_transit_motor.move(-127); // spin upper transit
    intake_transit_motor.move(127); // spin intake transit
    break;
  case OUTTAKE:
    upper_transit_motor.move(-127);  // spin upper transit motor
    intake_transit_motor.move(-127); // spin intake transit motor
    break;
  case STOPPED:
    upper_transit_motor.set_brake_mode(
        pros::MotorBrake::brake); // stop upper transit motor
    intake_transit_motor.set_brake_mode(
        pros::MotorBrake::brake); // stop intake transit motor
    upper_transit_motor.move(0);
    intake_transit_motor.move(0);
    break;
  }
}

// function to check if any controller buttons are pressed
void checkControllerButtonPress() {
  if (main_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
    funnel_engaged = !funnel_engaged;
  } 
  else if (main_controller.get_digital_new_press(
                 pros::CONTROLLER_R1)) {
    // for toggleable button
    if (current_ball_conveyor_state == MIDDLE_GOAL) {
      current_ball_conveyor_state = STOPPED;
    } else if (current_ball_conveyor_state == STOPPED) {
      current_ball_conveyor_state = MIDDLE_GOAL;
    } else {
      current_ball_conveyor_state = MIDDLE_GOAL;
    }
  } else if (main_controller.get_digital_new_press(pros::CONTROLLER_R2)) {
    // for toggelable button
    if (current_ball_conveyor_state == UPPER_GOAL) {
      current_ball_conveyor_state = STOPPED;
    } else if (current_ball_conveyor_state == STOPPED) {
      current_ball_conveyor_state = UPPER_GOAL;
    } else {
      // if in middle goal state, switch to upper goal state
      current_ball_conveyor_state = UPPER_GOAL;
    }
  }
    pros::lcd::print(0, "InFunnel: %s | Conveyer state: %s",
                     funnel_engaged ? "true" : "false",
                     current_ball_conveyor_state ? "true" : "false");
    }


double expo_joystick(int input) {
  // function to apply an exponential curve to the input.
  // is is applyed to the fowards and turn values in opcontrol()
  double normalized = (double)input / 127.0;       // normalize to [-1, 1]
  double curved = pow(fabs(normalized), EXPONENT); // apply exponential curve
  curved = curved * (normalized >= 0 ? 1 : -1);    // restore sign
  // if the output would be too large, dont change the number and pass it off to
  // the difference calculations
  if (curved >= 1 || curved <= -1) {
    return input;
  }
  return curved * 127.0; // scale back to motor range
}

void handleDrivetrainControl(int LEFT_Y_AXIS, int RIGHT_X_AXIS,
                             double &left_motor_voltage,
                             double &right_motor_voltage) {
  // if the turn value is not large unough, disregard and just use
  // Arcade control scheme
  // forward/backward on left stick Y
  // clockwise/counter-clockwise on right stick X

  // deadzone for joystick values
  if (abs(LEFT_Y_AXIS) < 5)
    LEFT_Y_AXIS = 0;
  if (abs(RIGHT_X_AXIS) < 5)
    RIGHT_X_AXIS = 0;

  // Constants
  constexpr int MOTOR_MAX = 127;
  // Overflow transfer correction
  double difference = 0.0;
  // easily changable scale factor
  double scale_factor = 1.7;

  // left motor overflow cases
  if (left_motor_voltage > MOTOR_MAX) {
    // Left is too positive (forward too strong)
    difference = (left_motor_voltage - MOTOR_MAX) / scale_factor;
    left_motor_voltage = MOTOR_MAX;
    right_motor_voltage += difference; // reduce right to preserve ratio
  } else if (left_motor_voltage < -MOTOR_MAX) {
    // Left is too negative (reverse too strong)
    difference = (left_motor_voltage + MOTOR_MAX) / scale_factor;
    left_motor_voltage = -MOTOR_MAX;
    right_motor_voltage += difference; // subtract because left is underflowing
  }

  // right motor overflow cases
  if (right_motor_voltage > MOTOR_MAX) {
    // Right is too positive (forward too strong)
    difference = (right_motor_voltage - MOTOR_MAX) / scale_factor;
    right_motor_voltage = MOTOR_MAX;
    left_motor_voltage += difference;
  } else if (right_motor_voltage < -MOTOR_MAX) {
    // Right is too negative (reverse too strong)
    difference = (right_motor_voltage + MOTOR_MAX) / scale_factor;
    right_motor_voltage = -MOTOR_MAX;
    left_motor_voltage += difference;
  }

  // double check to make sure we are in range
  left_motor_voltage = custom_clamp(left_motor_voltage, -MOTOR_MAX, MOTOR_MAX);
  right_motor_voltage =
      custom_clamp(right_motor_voltage, -MOTOR_MAX, MOTOR_MAX);
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
    // init variables for joystick values
    int LEFT_X_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_LEFT_X); // LEFT STICK X AXIS

    int LEFT_Y_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_LEFT_Y); // LEFT STICK Y AXIS

    int RIGHT_X_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_RIGHT_X); // RIGHT STICK X AXIS

    int RIGHT_Y_AXIS = main_controller.get_analog(
        pros::E_CONTROLLER_ANALOG_RIGHT_Y); // RIGHT STICK Y AXIS

    double left_motor_voltage = expo_joystick(
        LEFT_Y_AXIS + RIGHT_X_AXIS); // left motor voltage calculation
    double right_motor_voltage = expo_joystick(
        LEFT_Y_AXIS - RIGHT_X_AXIS); // right motor voltage calculation

    checkControllerButtonPress(); // Check if any controller buttons are pressed
    updatePneumatics();           // Update pneumatics based on bool/enum states
    updateBallConveyorMotors();
    handleDrivetrainControl(
        LEFT_Y_AXIS, RIGHT_X_AXIS, left_motor_voltage,
        right_motor_voltage); // Handle drive control and motor calc

    printf(0, "LV:%f|RV:%f", left_motor_voltage, right_motor_voltage);

    // print joystick values to controller screen for testing
    // controller screen for testing
    left_motors_drivetrain.move(left_motor_voltage);
    right_motors_drivetrain.move(right_motor_voltage);

    pros::delay(20); // keep update time set to keep cpu happy :)
  }
}