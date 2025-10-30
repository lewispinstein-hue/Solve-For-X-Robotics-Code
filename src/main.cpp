#include "auton.h"
#include "conveyor_handle.h"
#include "setup.h"

void initialize() {
  // calibrate lemlib stuff
  chassis.setPose(0, 0, 0);
  chassis.calibrate();
  // set the stopping mode for the motors
  left_motors_drivetrain.set_brake_mode(pros::MotorBrake::brake);
  right_motors_drivetrain.set_brake_mode(pros::MotorBrake::brake);
  upper_transit_motor.set_brake_mode(pros::MotorBrake::brake);
  intake_transit_motor.set_brake_mode(pros::MotorBrake::brake);
  // handle tests
  // handleSetupSelections();
  // run the acording tests
  if (testsToRun[0] == true) {
    clearScreen();
    handleSoftwareTests();
    clearScreen();
  }
  if (testsToRun[1] == true) {
    clearScreen();
    testPhysicals();
  }
  // runOdomCalibration();
  selectRoute();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  current_ball_conveyor_state = STOPPED;
  funnel_engaged = false;
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous
 * selector on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

// function to update conveyor motors based on enum states

// function to check if any controller buttons are pressed
int pnTimesPushed = 0;
void checkControllerButtonPress() {
  // handle pnuematics on button press
  if (main_controller.get_digital_new_press(
          Users::currentUser->getPnButton())) {
    funnel_engaged = !funnel_engaged;
    pnTimesPushed += 1;
    if (pnTimesPushed >= 20) {
      funnel_pneumatic_right.set_value(false);
      funnel_pneumatic_left.set_value(false);
    } else if (pnTimesPushed >= 17) {
      main_controller.rumble(".-.");
      funnel_pneumatic_right.set_value(funnel_engaged);
      funnel_pneumatic_left.set_value(funnel_engaged);
    } else {
      funnel_pneumatic_right.set_value(funnel_engaged);
      funnel_pneumatic_left.set_value(funnel_engaged);
    }
  } else if (main_controller.get_digital_new_press(
                 Users::currentUser->getSfMediumGoal())) {
    // for toggleable button
    setConveyorMotors(
        (current_ball_conveyor_state == MIDDLE_GOAL) ? STOPPED : MIDDLE_GOAL);
  } else if (main_controller.get_digital_new_press(
                 Users::currentUser->getSfHighGoal())) {
    // for toggelable button
    setConveyorMotors((current_ball_conveyor_state == UPPER_GOAL) ? STOPPED
                                                                  : UPPER_GOAL);
  } else if (main_controller.get_digital_new_press(
                 Users::currentUser->getSfBottomGoal())) {
    setConveyorMotors((current_ball_conveyor_state == OUTTAKE) ? STOPPED
                                                               : OUTTAKE);
  }

  // testing buttons for lemlib
  else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
    chassis.setPose(0, 0, 0);
    chassis.cancelAllMotions();
    main_controller.print(0, 0, "Pose reset to 0,0,0");
    selectRoute();
    autonomous();
  } else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
    // autonomousRoute1();
    chassis.setPose(0, 0, 0);
  } else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
    // runOdomCalibration();
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(180, 1000);
  }
}

// clamp function to replace std::clamp
double custom_clamp(double input, double MIN_VALUE, double MAX_VALUE) {
  if (input < MIN_VALUE)
    return MIN_VALUE;
  if (input > MAX_VALUE)
    return MAX_VALUE;
  return input;
}

// slew limit function
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
double EXPONENT_FOWARDS = 1.7;
double EXPONENT_TURN = 3;
int track_user = 1;
constexpr auto sizeOfUsers = 4;
void setActiveUser() {
  // make sure that the Users::currentUser contains a valid pointer
  if (Users::currentUser == nullptr) {
    // set the user to something usable
    Users::currentUser = &eli;
  }

  // check to see if we are trying to change the user
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
  // uptade the user adress based on selected user
  //  As we create users, put there adresses in this switch.
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

  // update variables
  MAX_DELTA = Users::currentUser->getSlewMax();
  MIN_DELTA = Users::currentUser->getSlewMin();
  scale_factor = Users::currentUser->getScaleFactor();
  EXPONENT_FOWARDS = Users::currentUser->getExponentFowards();
  EXPONENT_TURN = Users::currentUser->getExponentTurn();

  // we do not need to update certain members like the keybinds and the drive
  // types because there is nothing to update them to
}

double expo_joystick_foward(double input, double EXPONENT_FOWARDS) {
  // function to apply an exponential curve to the input.
  // is is applyed to the fowards and turn values in opcontrol()
  double normalized = (double)input / 127.0; // normalize to [-1, 1]
  double curved =
      pow(fabs(normalized), EXPONENT_FOWARDS);  // apply exponential curve
  curved = curved * (normalized >= 0 ? 1 : -1); // restore sign

  // if the output would be too large, dont change the number and pass it off
  // to the difference calculations
  if (curved >= 1 || curved <= -1) {
    return input;
  }
  return curved * 127.0; // scale back to motor range
}

double expo_joystick_turn(double input, double EXPONENT_TURN) {
  // function to apply an exponential curve to the input.
  // is is applyed to the fowards and turn values in opcontrol()
  double normalized = (double)input / 127.0; // normalize to [-1, 1]
  double curved =
      pow(fabs(normalized), EXPONENT_TURN);     // apply exponential curve
  curved = curved * (normalized >= 0 ? 1 : -1); // restore sign

  // if the output would be too large, dont change the number and pass it off
  // to the difference calculations
  if (curved >= 1 || curved <= -1) {
    return input;
  }
  return curved * 127.0; // scale back to motor range
}

double handleArcadeControl(double &left_motor_voltage,
                           double &right_motor_voltage, double scale_factor) {
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
  return left_motor_voltage + right_motor_voltage;
}

// forward declaration
void handleSlewControl(double &left_motor_voltage, double &right_motor_volatge);

void handleDriving(int LEFT_Y_AXIS, int RIGHT_X_AXIS, int RIGHT_Y_AXIS,
                   double &left_motor_voltage, double &right_motor_voltage) {

  // check which driver type the current user has
  if (Users::currentUser->getControlType() == Users::ControlType::Arcade) {
    handleArcadeControl(left_motor_voltage, right_motor_voltage,
                        scale_factor); // Handle drive control and motor calc

  } else if (Users::currentUser->getControlType() == Users::ControlType::Tank) {
    left_motor_voltage = expo_joystick_foward(LEFT_Y_AXIS, EXPONENT_FOWARDS);
    right_motor_voltage = expo_joystick_turn(RIGHT_Y_AXIS, EXPONENT_TURN);
  }
  handleSlewControl(left_motor_voltage, right_motor_voltage);
}

void handleSlewControl(double &left_motor_voltage,
                       double &right_motor_voltage) {
  // comparing cases for slew
  static double prev_left_voltage = 0;
  static double prev_right_voltage = 0;

  // slew control
  left_motor_voltage =
      slewLimit(left_motor_voltage, prev_left_voltage, MAX_DELTA, MIN_DELTA);
  right_motor_voltage =
      slewLimit(right_motor_voltage, prev_right_voltage, MAX_DELTA, MIN_DELTA);

  // update slew limiters
  prev_left_voltage = left_motor_voltage;
  prev_right_voltage = right_motor_voltage;

  // double check to make sure we are in range
  left_motor_voltage = custom_clamp(left_motor_voltage, -127, 127);
  right_motor_voltage = custom_clamp(right_motor_voltage, -127, 127);
}

void printDebug(double LEFT_Y_AXIS, double RIGHT_X_AXIS, float left_motor_v,
                float right_motor_v) {
  // set pen color and clear screen
  pros::screen::set_pen(pros::Color::white);
  // clearScreen();

  printToBrain(smallText, 25, 20, "Intake Funnel: %s",
               funnel_engaged ? "true" : "false");

  printToBrain(smallText, 25, 40, "Left Y: %.1f | Right X: %.1f", LEFT_Y_AXIS,
               RIGHT_X_AXIS);
  printToBrain(smallText, 25, 60, "LV: %.1f | RV: %.1f", left_motor_v,
               right_motor_v);
  // uptate user print
  printToBrain(smallText, 25, 80, "Current User: %s",
               Users::currentUser->getName().c_str());
  // print pos
  lemlib::Pose pose = chassis.getPose();

  // code to normalize the heading to 180 to -180 instead of the default total
  // degres value that getPose() returns
  double correctedHeading = pose.theta;
  correctedHeading = fmod(correctedHeading, 360.0);
  if (correctedHeading > 180) {
    correctedHeading -= 360;
  } else if (correctedHeading < -180) {
    correctedHeading += 360;
  }
  // print pose from lemlib
  printToBrain(smallText, 25, 100,
               "Times Pushed: %d | Corrected Heading: %.1f   ", pnTimesPushed,
               correctedHeading);
  printToBrain(smallText, 25, 120, "X Relative: %.2f | Y Relative: %.2f   ",
               pose.x, pose.y);
  // create variables for comparison
  static lemlib::Pose prevPose = chassis.getPose();
  static uint32_t prevTime = pros::millis();

  uint32_t currentTime = pros::millis();
  // make velocity calculation
  double dt = (currentTime - prevTime) / 1000.0;
  double vx = (dt > 0) ? (pose.x - prevPose.x) / dt : 0;
  double vy = (dt > 0) ? (pose.y - prevPose.y) / dt : 0;

  // update timers
  prevPose = pose;
  prevTime = currentTime;
  // print results
  printToBrain(smallText, 25, 140, "X Vel: %.2f | ", vx);
  printToBrain(smallText, 140, 140, "Y Vel: %.2f", vy);
  //--------------Prints for controller screen----------------------//

  char buffer[21]; // character limit of controller display
  sprintf(buffer, "H: %.1f|X: %.1f|Y: %.1f", correctedHeading, pose.x, pose.y);
  printf("Buffer: %s", buffer); // trying to print to console
  main_controller.print(0, 0, buffer);
}

/**
 * Runs the user autonomous code. This function will be started in its own
 * task with the default priority and stack size whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the
 * autonomous mode. Alternatively, this function may be called in initialize
 * or opcontrol for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start
 * it from where it left off.
 */

void opcontrol() {
  while (true) {
    // init variables for joystick values
    double LEFT_Y_AXIS = main_controller.get_analog(CONTROLLER_LEFT_Y);
    double RIGHT_X_AXIS = main_controller.get_analog(CONTROLLER_RIGHT_X);
    double RIGHT_Y_AXIS = main_controller.get_analog(CONTROLLER_RIGHT_Y);
    // deadzone for joystick values
    if (fabs(LEFT_Y_AXIS) < 5)
      LEFT_Y_AXIS = 0;
    if (fabs(RIGHT_X_AXIS) < 5)
      RIGHT_X_AXIS = 0;

    double forward = expo_joystick_foward(LEFT_Y_AXIS, EXPONENT_FOWARDS);
    double turn = expo_joystick_turn(RIGHT_X_AXIS, EXPONENT_TURN);
    double left_motor_voltage = forward + turn;
    double right_motor_voltage = forward - turn;

    setActiveUser();
    checkControllerButtonPress(); // Check if any controller buttons are
                                  // pressed
    handleDriving(LEFT_Y_AXIS, RIGHT_X_AXIS, RIGHT_Y_AXIS, left_motor_voltage,
                  right_motor_voltage);

    left_motors_drivetrain.move(left_motor_voltage);
    right_motors_drivetrain.move(right_motor_voltage);

    printDebug(LEFT_Y_AXIS, RIGHT_X_AXIS, left_motor_voltage,
               right_motor_voltage);

    pros::delay(20); // keep update time set to keep cpu happy :)
  }
}