#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cstdlib>
#include <string>

// unused headers:
//  #include "pros/rotation.h"
//  #include "pros/rotation.hpp"
//  #include "pros/motors.h"
//  #include "lemlib/chassis/odom.hpp"
//  #include "lemlib/api.hpp"
//  #include "liblvgl/llemu.hpp"
//  #include "pros/llemu.hpp"

// defines for controller buttons for readability
#define CONTROLLER_UP pros::E_CONTROLLER_DIGITAL_UP
#define CONTROLLER_DOWN pros::E_CONTROLLER_DIGITAL_DOWN

// define joysticks
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
constexpr float HALF_TRACK = 7.5f / 2.0f;

pros::Imu inertial_sensor(16); // adjust port when we get sensor

lemlib::TrackingWheel
    leftVerticalTrackingWheel(&left_motors_drivetrain,
                              lemlib::Omniwheel::NEW_325,
                              HALF_TRACK, // flip sign if numbers are off
                              600);

lemlib::TrackingWheel
    rightVerticalTrackingWheel(&right_motors_drivetrain,
                               lemlib::Omniwheel::NEW_325,
                               -HALF_TRACK, // flip sign if numbers are off
                               600);

lemlib::ControllerSettings lateralSettings(12, 0, 2, // kP, kI, kD
                                           0,      // integral anti-windup range
                                           1, 100, // small error range, timeout
                                           3, 500, // large error range, timeout
                                           30      // max acceleration (slew)
);

lemlib::ControllerSettings angularSettings(4, 0, 10, 0, 1, 0, 0, 0, 30);

lemlib::OdomSensors
    odomSensors(&leftVerticalTrackingWheel,  // vertical1
                &rightVerticalTrackingWheel, // vertical2
                nullptr,         // horizontal1 (no horizontal tracking wheel)
                nullptr,         // horizontal2
                &inertial_sensor // IMU
    );

lemlib::Drivetrain main_drivetrain(
    &left_motors_drivetrain,  // left motor group
    &right_motors_drivetrain, // right motor group
    7.5, // track width in inches (measure center-to-center of wheels)
    lemlib::Omniwheel::NEW_325, // 3.25" omni wheels
    900, // wheel RPM (green cartridge = 200, blue = 600, red = 100)
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
  chassis.setPose(0, 0, 0);
  chassis.calibrate();
  left_motors_drivetrain.set_brake_mode(pros::MotorBrake::brake);
  right_motors_drivetrain.set_brake_mode(pros::MotorBrake::brake);

  current_ball_conveyor_state = STOPPED; // initial state
  funnel_engaged = false;
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
  pros::controller_digital_e_t sf_high_goal;   // button for high goal conveyor
  pros::controller_digital_e_t sf_medium_goal; // buttor for low goal conveyor
  pros::controller_digital_e_t
      sf_bottom_goal;                     // button for bottom goal conveyor
  pros::controller_digital_e_t pn_button; // button for pneumatic toggle

public:
  // constructor
  Users(std::string name, int slew_max, int slew_min, double exponent,
        double scale_factor, ControlType control,
        pros::controller_digital_e_t sf_high_goal,
        pros::controller_digital_e_t sf_medium_goal,
        pros::controller_digital_e_t sf_bottom_goal,
        pros::controller_digital_e_t pn_button)
      : name(name), SLEW_MAX(slew_max), SLEW_MIN(slew_min),
        SCALE_FACTOR(scale_factor), EXPONENT(exponent), control_type(control),
        sf_high_goal(sf_high_goal), sf_medium_goal(sf_medium_goal),
        sf_bottom_goal(sf_bottom_goal), pn_button(pn_button) {}

  // setter
  void setDriverInfo(std::string newName, int slew_max, int slew_min,
                     double exponent, double scale_factor, ControlType control,
                     pros::controller_digital_e_t sf_high_goal,
                     pros::controller_digital_e_t sf_medium_goal,
                     pros::controller_digital_e_t sf_bottom_goal,
                     pros::controller_digital_e_t pn_button) {
    this->name = newName;
    this->SLEW_MAX = slew_max;
    this->SLEW_MIN = slew_min;
    this->EXPONENT = exponent;
    this->SCALE_FACTOR = scale_factor;
    this->control_type = control;
    this->sf_high_goal = sf_high_goal;
    this->sf_medium_goal = sf_medium_goal;
    this->sf_bottom_goal = sf_bottom_goal;
    this->pn_button = pn_button;
  }

  // getters
  std::string getName() const { return name; }
  int getSlewMax() const { return SLEW_MAX; }
  int getSlewMin() const { return SLEW_MIN; }
  double getExponent() const { return EXPONENT; }
  double getScaleFactor() const { return SCALE_FACTOR; }
  ControlType getControlType() const { return control_type; }

  // getters for keybinds
  pros::controller_digital_e_t getSfHighGoal() const { return sf_high_goal; }
  pros::controller_digital_e_t getSfMediumGoal() const {
    return sf_medium_goal;
  }
  pros::controller_digital_e_t getSfBottomGoal() const {
    return sf_bottom_goal;
  }
  pros::controller_digital_e_t getPnButton() const { return pn_button; }

  static Users *currentUser;
};
// setting order: name, slew acceleration, slew decelleration, exponent, scale
// factor. keybinds: high goal, medium goal, low goal, pneumatic button

Users eli("Eli  ", 50, 20, 1.9, 1.6, Users::ControlType::Arcade,
          pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2,
          pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_B);

Users lewis("Lewis", 50, 50, 2, 1.3, Users::ControlType::Arcade,
            pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2,
            pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_B);

Users ian("Ian", 40, 40, 2.1, 1.7, Users::ControlType::Arcade,
          pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2,
          pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_B);

Users sanjith("Sanjith", 20, 30, 2, 1.2, Users::ControlType::Arcade,
              pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2,
              pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_B);

Users TEST_USER("TEST_USER", 20, 30, 3, 1.2, Users::ControlType::Tank,
                pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2,
                pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_B);
Users *Users::currentUser = &eli; // globally initialize current user as default

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

// function to update conveyor motors based on enum states
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
  // handle pnuematics on button press
  if (main_controller.get_digital_new_press(
          Users::currentUser->getPnButton())) {
    funnel_engaged = !funnel_engaged;
    funnel_pneumatic_right.set_value(funnel_engaged);
    funnel_pneumatic_left.set_value(funnel_engaged);
  } else if (main_controller.get_digital_new_press(
                 Users::currentUser->getSfMediumGoal())) {
    // for toggleable button
    current_ball_conveyor_state =
        (current_ball_conveyor_state == MIDDLE_GOAL) ? STOPPED : MIDDLE_GOAL;
  } else if (main_controller.get_digital_new_press(
                 Users::currentUser->getSfHighGoal())) {
    // for toggelable button
    current_ball_conveyor_state =
        (current_ball_conveyor_state == UPPER_GOAL) ? STOPPED : UPPER_GOAL;
  } else if (main_controller.get_digital_new_press(
                 Users::currentUser->getSfBottomGoal())) {
    current_ball_conveyor_state =
        (current_ball_conveyor_state == OUTTAKE) ? STOPPED : OUTTAKE;
  }

  else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
    chassis.setPose(0, 0, 0);
    chassis.cancelAllMotions();
    main_controller.print(0, 0, "Pose reset to 0,0,0");
  } else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
    chassis.turnToHeading(chassis.getPose().theta + 90, 1000);
  } else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
    chassis.swingToHeading(100, lemlib::DriveSide::LEFT, 5000);
  }
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
  double absTarget = fabs(target);
  double absPrev = fabs(prev);
  double absDelta = fabs(delta);

  // Determine if increasing or decreasing in magnitude
  bool increasingMagnitude = absTarget > absPrev;

  // Apply correct limit
  if (increasingMagnitude) {
    // accelerating
    if (absDelta > riseMaxDelta) {
      absDelta = riseMaxDelta;
    }
  } else {
    // decelerating
    if (absDelta > fallMaxDelta) {
      absDelta = fallMaxDelta;
    }
  }

  // Reapply original sign
  double limitedDelta = (delta >= 0) ? absDelta : -absDelta;

  return prev + limitedDelta;
}

// create variales for Users with default values
double MAX_DELTA = 2;
double MIN_DELTA = 2;
double scale_factor = 2;
double EXPONENT = 1.7;

int track_user = 1;
constexpr auto sizeOfUsers = 5;
void setActiveUser() {
  if (main_controller.get_digital_new_press(CONTROLLER_UP) &&
      main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
      main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    track_user++;
    if (track_user > sizeOfUsers)
      track_user = 1; // wrap around
  } else if (main_controller.get_digital_new_press(CONTROLLER_DOWN) &&
             main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
             main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    track_user--;
    if (track_user < 0)
      track_user = sizeOfUsers; // wrap around
  }
  // As we create users, put there adresses in this switch.
  switch (track_user) {
  case 1:
    Users::currentUser = &eli;
    break;
  case 2:
    Users::currentUser = &lewis;
    break;
  case 3:
    Users::currentUser = &ian;
    break;
  case 4:
    Users::currentUser = &sanjith;
    break;
  default:
    Users::currentUser = &eli;
    break;
  }

  if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
      main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
      main_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    Users::currentUser = &TEST_USER;
  }

  // update variables
  MAX_DELTA = Users::currentUser->getSlewMax();
  MIN_DELTA = Users::currentUser->getSlewMin();
  scale_factor = Users::currentUser->getScaleFactor();
  EXPONENT = Users::currentUser->getExponent();
  // we do not need to update certain members like the keybinds and the drive
  // types because there is nothing to update them to
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

void handleDriving(int LEFT_Y_AXIS, int RIGHT_X_AXIS, int RIGHT_Y_AXIS,
                   double &left_motor_voltage, double &right_motor_voltage) {

  if (Users::currentUser->getControlType() == Users::ControlType::Arcade) {
    handleArcadeControl(
        left_motor_voltage,
        right_motor_voltage); // Handle drive control and motor calc
  } else if (Users::currentUser->getControlType() == Users::ControlType::Tank) {
    left_motor_voltage = expo_joystick(LEFT_Y_AXIS);
    right_motor_voltage = expo_joystick(RIGHT_Y_AXIS);
  }
}

void printDebug(double LEFT_Y_AXIS, double RIGHT_X_AXIS, float left_motor_v,
                float right_motor_v) {

  pros::screen::set_pen(pros::Color::white);
  // pros::screen::erase();

  pros::screen::print(pros::E_TEXT_SMALL, 25, 20, "Intake Funnel: %s",
                      funnel_engaged ? "true" : "false");

  pros::screen::print(pros::E_TEXT_SMALL, 25, 40,
                      "Left Y: %.1f | Right X: %.1f   ", LEFT_Y_AXIS,
                      RIGHT_X_AXIS);
  pros::screen::print(pros::E_TEXT_SMALL, 25, 60, "LV: %.1f | RV: %.1f   ",
                      left_motor_v, right_motor_v);
  // uptate user print
  pros::screen::print(pros::E_TEXT_SMALL, 25, 80, "Current User: %s    ",
                      Users::currentUser->getName().c_str());
  // print pos
  lemlib::Pose pose = chassis.getPose();

  // code to normalize the heading to 180 to -180 instead of the default total
  // degres value that getPos() returns

  double correctedHeading = pose.theta;
  correctedHeading = fmod(correctedHeading, 360.0);
  if (correctedHeading > 180) {
    correctedHeading -= 360;
  } else if (correctedHeading < -180) {
    correctedHeading += 360;
  }

  pros::screen::print(pros::E_TEXT_SMALL, 25, 100,
                      "Heading: %.2f | Corrected Heading: %.1f   ", pose.theta,
                      correctedHeading);
  pros::screen::print(pros::E_TEXT_SMALL, 25, 120,
                      "X Relative: %.2f | Y Relative: %.2f   ", pose.x, pose.y);

  static lemlib::Pose prevPose = chassis.getPose();
  static uint32_t prevTime = pros::millis();

  uint32_t currentTime = pros::millis();
  lemlib::Pose currentPose = chassis.getPose();

  double dt = (currentTime - prevTime) / 1000.0;
  double vx = (dt > 0) ? (currentPose.x - prevPose.x) / dt : 0;
  double vy = (dt > 0) ? (currentPose.y - prevPose.y) / dt : 0;

  prevPose = currentPose;
  prevTime = currentTime;

  pros::screen::print(pros::E_TEXT_SMALL, 25, 140, "X Vel: %.2f | ", vx);
  pros::screen::print(pros::E_TEXT_SMALL, 140, 140, "Y Vel: %.2f", vy);
  //--------------Prints for controller screen----------------------//

  char buffer[21]; // character limit of controller display
  sprintf(buffer, "H: %.1f|X: %.1f|Y: %.1f", correctedHeading, pose.x, pose.y);
  main_controller.print(0, 0, buffer);
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
    // make sure that the Users::currentUser contains a valid pointer
    if (Users::currentUser == nullptr) {
      pros::screen::print(
          pros::E_TEXT_SMALL, 100, 300,
          "ERROR. NULLPTR ON POINTER 'Users::currentUser'. EXITING");
      while (true) {
        pros::delay(100); // prevent resets, keep message on screen
      }
    }
    // init variables for joystick values
    int LEFT_Y_AXIS = main_controller.get_analog(CONTROLLER_LEFT_Y);
    int RIGHT_X_AXIS = main_controller.get_analog(CONTROLLER_RIGHT_X);
    int RIGHT_Y_AXIS = main_controller.get_analog(CONTROLLER_RIGHT_Y);

    // deadzone for joystick values
    if (abs(LEFT_Y_AXIS) < 5)
      LEFT_Y_AXIS = 0;
    if (abs(RIGHT_X_AXIS) < 5)
      RIGHT_X_AXIS = 0;

    double forward = expo_joystick(LEFT_Y_AXIS);
    double turn = expo_joystick(RIGHT_X_AXIS);
    double left_motor_voltage = forward + turn;
    double right_motor_voltage = forward - turn;

    checkControllerButtonPress(); // Check if any controller buttons are pressed
    updateBallConveyorMotors();   // handle the enum that controlls the ball
                                  // conveyors
    setActiveUser();
    handleDriving(LEFT_Y_AXIS, RIGHT_X_AXIS, RIGHT_Y_AXIS, left_motor_voltage,
                  right_motor_voltage);

    // comparing cases for slew
    static double prev_left_voltage = 0;
    static double prev_right_voltage = 0;

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

    printDebug(LEFT_Y_AXIS, RIGHT_X_AXIS, left_motor_voltage,
               right_motor_voltage);

    pros::delay(20); // keep update time set to keep cpu happy :)
  }
}