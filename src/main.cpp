#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <string>

// unused headers:
//  #include "pros/rotation.h"
//  #include "pros/rotation.hpp"
//  #include "pros/motors.h"
//  #include "lemlib/chassis/odom.hpp"
//  #include "lemlib/api.hpp"

// defines for controller buttons for readability
#define CONTROLLER_R1 pros::E_CONTROLLER_DIGITAL_R1
#define CONTROLLER_R2 pros::E_CONTROLLER_DIGITAL_R2
#define CONTROLLER_L1 pros::E_CONTROLLER_DIGITAL_L1
#define CONTROLLER_L2 pros::E_CONTROLLER_DIGITAL_L2
#define CONTROLLER_B pros::E_CONTROLLER_DIGITAL_B
#define CONTROLLER_LEFT pros::E_CONTROLLER_DIGITAL_LEFT
#define CONTROLLER_RIGHT pros::E_CONTROLLER_DIGITAL_RIGHT
#define CONTROLLER_UP pros::E_CONTROLLER_DIGITAL_UP
#define CONTROLLER_DOWN pros::E_CONTROLLER_DIGITAL_DOWN

// define joysticks
#define CONTROLLER_LEFT_X pros::E_CONTROLLER_ANALOG_LEFT_X
#define CONTROLLER_LEFT_Y pros::E_CONTROLLER_ANALOG_LEFT_Y
#define CONTROLLER_RIGHT_X pros::E_CONTROLLER_ANALOG_RIGHT_X
#define CONTROLLER_RIGHT_Y pros::E_CONTROLLER_ANALOG_RIGHT_Y

// init drivetrain motor groups and controller.
// sets motor to blue gear cartridge, inits ports, and starts tracking the
// encoding for the motors in degrees
pros::MotorGroup left_motors_drivetrain({-6, 13, -14},
                                        pros::v5::MotorGears::rpm_600,
                                        pros::v5::MotorUnits::degrees);
pros::MotorGroup right_motors_drivetrain({8, 18, -20},
                                         pros::v5::MotorGears::rpm_600,
                                         pros::v5::MotorUnits::degrees);
pros::Controller main_controller(pros::E_CONTROLLER_MASTER);

// pneumatics
bool funnel_engaged = false;
pros::adi::Pneumatics funnel_pneumatic_right('H', false);
pros::adi::Pneumatics funnel_pneumatic_left('G', false);

// intake and transit motors
pros::Motor upper_transit_motor(7);
pros::Motor intake_transit_motor(-2);

// Lemlib initialization
pros::Imu inertial_sensor(10); // adjust port when we get sensor

lemlib::ControllerSettings lateralSettings(12, 0, 2, // kP, kI, kD
                                           0,      // integral anti-windup range
                                           1, 100, // small error range, timeout
                                           3, 500, // large error range, timeout
                                           5       // max acceleration (slew)
);

lemlib::ControllerSettings angularSettings(3, 0, 12, 0, 1, 100, 3, 500, 5);

lemlib::OdomSensors odomSensors(nullptr,         // vertical1
                                nullptr,         // vertical2
                                nullptr,         // horizontal1
                                nullptr,         // horizontal2
                                &inertial_sensor // IMU
);

lemlib::Drivetrain main_drivetrain(
    &left_motors_drivetrain,  // left motor group
    &right_motors_drivetrain, // right motor group
    11.5, // track width in inches (measure center-to-center of wheels)
    lemlib::Omniwheel::NEW_325, // 3.25" omni wheels
    600, // wheel RPM (green cartridge = 200, blue = 600, red = 100)
    2    // chase power (leave as 2 unless tuning)
);

lemlib::Chassis chassis(main_drivetrain, lateralSettings, angularSettings,
                        odomSensors);

// enum for ball conveyer belt
enum ball_conveyor_state {
  UPPER_GOAL,  // intake balls || spin both up
  MIDDLE_GOAL, // outtake balls || spin INTAKE_TRANSIST up, UPPER_TRANSIT down
  OUTTAKE,     // outtake balls || spin both down
  STOPPED      // stop both motors
};

ball_conveyor_state current_ball_conveyor_state;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  chassis.resetLocalPosition();
  left_motors_drivetrain.set_brake_mode(pros::MotorBrake::brake);
  right_motors_drivetrain.set_brake_mode(pros::MotorBrake::brake);

  current_ball_conveyor_state = STOPPED; // initial state
  funnel_engaged = false;

  pros::lcd::initialize();
}

class Users {
public:
  enum class ControlType { Arcade, Tank };

protected:
  std::string name;
  int SLEW_MAX;
  int SLEW_MIN;
  double SCALE_FACTOR;
  double EXPONENT;
  ControlType control_type;

public:
  //  constructor
  Users(std::string name, int slew_max, int slew_min, double exponent,
        double scale_factor, ControlType control)
      : name(name), SLEW_MAX(slew_max), SLEW_MIN(slew_min),
        SCALE_FACTOR(scale_factor), EXPONENT(exponent), control_type(control) {}

  // setter
  void setDriverInfo(std::string newName, int slew_max, int slew_min,
                     double exponent, double scale_factor,
                     ControlType control) {
    this->name = newName;
    this->SLEW_MAX = slew_max;
    this->SLEW_MIN = slew_min;
    this->EXPONENT = exponent;
    this->SCALE_FACTOR = scale_factor;
    this->control_type = control;
  }

  // getters
  std::string getName() const { return name; }
  int getSlewMax() const { return SLEW_MAX; }
  int getSlewMin() const { return SLEW_MIN; }
  double getExponent() const { return EXPONENT; }
  double getScaleFactor() const { return SCALE_FACTOR; }
  ControlType getControlType() const { return control_type; }

  static Users *currentUser;
};

Users eli("Eli", 8, 12, 1.8, 1.6, Users::ControlType::Arcade);
Users *Users::currentUser = &eli; // globally initialize current user

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
void competition_initialize() { chassis.resetLocalPosition(); }

// function to update pneumatics based on states
void updatePneumatics() {
  if (funnel_engaged) {
    // engage funnel pneumatics when true
    funnel_pneumatic_right.set_value(true);
    funnel_pneumatic_left.set_value(false);
  } else if (!funnel_engaged) {
    // disengage funnel pneumatics when false
    funnel_pneumatic_right.set_value(false);
    funnel_pneumatic_left.set_value(true);
  }
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
  if (main_controller.get_digital_new_press(CONTROLLER_B)) {
    funnel_engaged = !funnel_engaged;
  } else if (main_controller.get_digital_new_press(CONTROLLER_R1)) {
    // for toggleable button
    if (current_ball_conveyor_state == MIDDLE_GOAL) {
      current_ball_conveyor_state = STOPPED;
    } else if (current_ball_conveyor_state == STOPPED) {
      current_ball_conveyor_state = MIDDLE_GOAL;
    } else {
      current_ball_conveyor_state = MIDDLE_GOAL;
    }
  } else if (main_controller.get_digital_new_press(CONTROLLER_R2)) {
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
  printf("Intake funnel Funnel: %s | Conveyer state: %s",
         funnel_engaged ? "true" : "false",
         current_ball_conveyor_state ? "true" : "false");
}

double custom_clamp(double input, double MIN_VALUE, double MAX_VALUE) {
  if (input < MIN_VALUE)
    return MIN_VALUE;
  if (input > MAX_VALUE)
    return MAX_VALUE;
  return input;
}

double slewLimit(double target, double prev, double riseMaxDelta,
                 double fallMaxDelta) {
  double delta = target - prev;

  // accelerating (increasing magnitude)
  if (delta > 0) {
    if (delta > riseMaxDelta)
      delta = riseMaxDelta;
  }
  // decelerating (reducing magnitude)
  else if (delta < 0) {
    if (delta < -fallMaxDelta)
      delta = -fallMaxDelta;
  }
  return prev + delta;
}

// create variales for Users
double MAX_DELTA = 2;
double MIN_DELTA = 2;
double scale_factor = 2;
double EXPONENT = 1.7;
void setActiveUser() {
  if (main_controller.get_digital_new_press(CONTROLLER_UP)) {
    Users::currentUser = &eli;
  }
  // update variables
  MAX_DELTA = Users::currentUser->getSlewMax();
  MIN_DELTA = Users::currentUser->getSlewMin();
  scale_factor = Users::currentUser->getScaleFactor();
  EXPONENT = Users::currentUser->getExponent();
  // uptate user print
  pros::lcd::print(4, "Current User: %s", *Users::currentUser);
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

void handleArcadeControl(double &left_motor_voltage,
                         double &right_motor_voltage) {
  // if the turn value is not large unough, disregard and just use
  // Arcade control scheme
  // forward/backward on left stick Y
  // clockwise/counter-clockwise on right stick X

  // Constants
  constexpr int MOTOR_MAX = 127;
  // Overflow transfer correction
  double difference = 0.0;

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
    int LEFT_Y_AXIS = main_controller.get_analog(CONTROLLER_LEFT_Y);
    int RIGHT_X_AXIS = main_controller.get_analog(CONTROLLER_RIGHT_X);
    int RIGHT_Y_AXIS = main_controller.get_analog(CONTROLLER_RIGHT_Y);

    // deadzone for joystick values
    if (abs(LEFT_Y_AXIS) < 5) {
      LEFT_Y_AXIS = 0;
      left_motors_drivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      left_motors_drivetrain.move(0);
    }
    if (abs(RIGHT_X_AXIS) < 5) {
      RIGHT_X_AXIS = 0;
      right_motors_drivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      right_motors_drivetrain.move(0);
    }

    double left_motor_voltage = expo_joystick(
        LEFT_Y_AXIS + RIGHT_X_AXIS); // left motor voltage calculation
    double right_motor_voltage = expo_joystick(
        LEFT_Y_AXIS - RIGHT_X_AXIS); // right motor voltage calculation

    checkControllerButtonPress(); // Check if any controller buttons are
                                  // pressed
    updatePneumatics();           // Update pneumatics based on bool/enum states
    updateBallConveyorMotors();
    setActiveUser();

    if (Users::currentUser->getControlType() == Users::ControlType::Arcade) {
      handleArcadeControl(
          left_motor_voltage,
          right_motor_voltage); // Handle drive control and motor calc
    } else if (Users::currentUser->getControlType() ==
               Users::ControlType::Tank) {
      left_motor_voltage = expo_joystick(LEFT_Y_AXIS);
      right_motor_voltage = expo_joystick(RIGHT_Y_AXIS);
    }

    // comparing cases for slew
    static double prev_left_voltage = 0.0;
    static double prev_right_voltage = 0.0;

    // slew control
    left_motor_voltage =
        slewLimit(left_motor_voltage, prev_left_voltage, MAX_DELTA, MIN_DELTA);
    right_motor_voltage = slewLimit(right_motor_voltage, prev_right_voltage,
                                    MAX_DELTA, MIN_DELTA);

    // update slew limiters
    prev_left_voltage = left_motor_voltage;
    prev_right_voltage = right_motor_voltage;

    // double check to make sure we are in range
    left_motor_voltage = custom_clamp(left_motor_voltage, -127, 127);
    right_motor_voltage = custom_clamp(right_motor_voltage, -127, 127);

    left_motors_drivetrain.move(left_motor_voltage);
    right_motors_drivetrain.move(right_motor_voltage);

    pros::lcd::print(1, "Right Y: %d || Left X: %d", LEFT_Y_AXIS, RIGHT_X_AXIS);
    pros::lcd::print(2, "Right Y: %f || Left X: %f", left_motor_voltage,
                     right_motor_voltage);

    pros::delay(20); // keep update time set to keep cpu happy :)
  }
}