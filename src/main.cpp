#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/screen.hpp"

// my helper files
#include "conveyor_handle.h"
#include "test_class.h"
#include "users_class.h"

// defines for controller buttons for readability
#define CONTROLLER_UP pros::E_CONTROLLER_DIGITAL_UP
#define CONTROLLER_DOWN pros::E_CONTROLLER_DIGITAL_DOWN

// more reader friendly print command
#define printToBrain pros::screen::print
#define smallText pros::E_TEXT_SMALL

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
pros::adi::Pneumatics funnel_pneumatic_right('H', funnel_engaged);
pros::adi::Pneumatics funnel_pneumatic_left('G', funnel_engaged);

// Lemlib initialization

// the length of our robot from furthest wheel to closest wheel on the same side
constexpr float TRACK_WIDTH = 11.5;

pros::Imu imu(16); // the slot for our imu

// tracking wheels are the built in IME's in the motors
lemlib::TrackingWheel leftVerticalTrackingWheel(&left_motors_drivetrain,
                                                lemlib::Omniwheel::NEW_325,
                                                (TRACK_WIDTH / 2), 540);

lemlib::TrackingWheel rightVerticalTrackingWheel(&right_motors_drivetrain,
                                                 lemlib::Omniwheel::NEW_325,
                                                 ((-TRACK_WIDTH) / 2), 540);

// sensor init with the sensors we created above
lemlib::OdomSensors
    odomSensors(&leftVerticalTrackingWheel,  // vertical1
                &rightVerticalTrackingWheel, // vertical2
                nullptr, // horizontal1 (no horizontal tracking wheel)
                nullptr, // horizontal2
                &imu     // IMU
    );

// settings lemlib uses (need to be tweaked)
lemlib::ControllerSettings lateralSettings(6, 0, 10, // kP, kI, kD
                                           0,      // integral anti-windup range
                                           1, 100, // small error range, timeout
                                           3, 500, // large error range, timeout
                                           0       // max acceleration (slew)
);

lemlib::ControllerSettings angularSettings(4, 0, 10, 0, 1, 100, 3, 500, 0);

// creating drivedrain object te be used in chassis
lemlib::Drivetrain main_drivetrain(
    &left_motors_drivetrain,  // left motor group
    &right_motors_drivetrain, // right motor group
    TRACK_WIDTH, // track width in inches (measure center-to-center of wheels)
    lemlib::Omniwheel::NEW_325, // 3.25" omni wheels
    400, // wheel RPM (green cartridge = 200, blue = 600, red = 100)
    2    // chase power (leave as 2 unless tuning)
);

// creating chassis for odometry and PID
lemlib::Chassis chassis(main_drivetrain, lateralSettings, angularSettings,
                        odomSensors);

// foward declaration for tests
double expo_joystick_foward(double, double);
double custom_clamp(double, double, double);
double handleArcadeControl(double &, double &, double);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void clearScreen() {
  // just trying everything to clear the screen.
  pros::screen::erase_rect(0, 0, 480, 272);
  pros::screen::fill_rect(0, 0, 480, 272);
  pros::screen::erase();
}

void handleTests() {
  // for floating point inacuracies
  constexpr double TOLERANCE = 0.05;
  // tracks the return value of runTestsArgs()
  int tests_passed;
  // Initialize Test objects

  // test for expo_joystick_foward
  Test testExpoJoystick({1, 64, 63, 127}, {1, 1.2, 3.1, 2}, {},
                        {1.0, 55.8, 14.5, 127.0}, "Expo Joystck",
                        expo_joystick_foward, TOLERANCE);
  tests_passed += testExpoJoystick.runTestsArgs2(500, true);
  // tesing custom clamp
  Test customClampTest({305, -102, 491, -230}, {200, -400, 506, -215},
                       {400, -200, 609, -102}, {305, -200, 506, -215},
                       "Custom Clamp", custom_clamp, TOLERANCE);
  tests_passed += customClampTest.runTestsArgs3(500, true);

  std::vector<double> leftYArcade = {127, -127, 127, 0, 0, 20, 50, 35};
  std::vector<double> rightXArcade = {0, 0, 127, 127, -127, 0, 50, 80};
  std::vector<double> testingScaleFactor(rightXArcade.size(), 2);
  std::vector<double> expectedOutArcade = {254, -254, 191, 0, 0, 20, 100, 65};

  Test testArcadeControl(leftYArcade, rightXArcade, testingScaleFactor,
                         expectedOutArcade, "Arcade Control",
                         handleArcadeControl, TOLERANCE);
  tests_passed += testArcadeControl.runTestsArgs3(500, true);

  // print summary
  clearScreen(); // clear screen
  printToBrain(smallText, 25, 0.0, "Summary: ");
  printToBrain(smallText, 25, 20, "Tests run: 3");
  printToBrain(smallText, 25, 40, "Tests passed: %d/16", tests_passed);
  printToBrain(smallText, 25, 60,
               "Would you like to run tests again? (A = y/X = n)");

  while (true) {
    if (main_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      clearScreen();
      printToBrain(smallText, 25, 40, "Running Tests again...");
      pros::delay(300);
      testExpoJoystick.runTestsArgs2(3000, true);
      customClampTest.runTestsArgs3(3000, true);
      testArcadeControl.runTestsArgs3(3000, true);
      return;
    } else if (main_controller.get_digital_new_press(
                   pros::E_CONTROLLER_DIGITAL_X)) {
      clearScreen();
      printToBrain(smallText, 25, 40, "Continuing program...");
      return;
    }
  }
}

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
  //   handleTests();
  //   clearScreen();
}

// visit users_class.h for User setup explanation
Users eli("Eli     ", 25, 40, 1.8, 1.6, 3, Users::ControlType::Arcade,
          pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_R1,
          pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_B);

Users lewis("Lewis", 25, 40, 1.9, 1.4, 3, Users::ControlType::Arcade,
            pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_R1,
            pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_B);

Users ian("Ian", 20, 30, 2.1, 1.5, 3, Users::ControlType::Arcade,
          pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_R1,
          pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_A);

Users sanjith("Sanjith", 25, 40, 2.1, 1.5, 3, Users::ControlType::Arcade,
              pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2,
              pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_B);

Users *Users::currentUser = &sanjith; // globally initialize default user

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
 * competition-specific initialization routines, such as an autonomous
 * selector on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize(){};

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
    updateBallConveyorMotors(
        ((current_ball_conveyor_state == MIDDLE_GOAL) ? STOPPED : MIDDLE_GOAL));
  } else if (main_controller.get_digital_new_press(
                 Users::currentUser->getSfHighGoal())) {
    // for toggelable button
    updateBallConveyorMotors(
        (current_ball_conveyor_state == UPPER_GOAL) ? STOPPED : UPPER_GOAL);
  } else if (main_controller.get_digital_new_press(
                 Users::currentUser->getSfBottomGoal())) {
    updateBallConveyorMotors(
        (current_ball_conveyor_state == OUTTAKE) ? STOPPED : OUTTAKE);
  }

  // testing buttons for lemlib
  else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
    chassis.setPose(0, 0, 0);
    chassis.cancelAllMotions();
    main_controller.print(0, 0, "Pose reset to 0,0,0");
  } else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
  } else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
    autonomous();
  }
}

void testPhysicals() {
  clearScreen();
  printToBrain(smallText, 25, 20, "Running Physical Tests...       ");
  while (true) {
    printToBrain(smallText, 25, 20, "Testing Conveyor Motors...         ");
    updateBallConveyorMotors(OUTTAKE);
    pros::delay(500);
    updateBallConveyorMotors(UPPER_GOAL);
    pros::delay(500);
    updateBallConveyorMotors(MIDDLE_GOAL);
    pros::delay(500);
    updateBallConveyorMotors(STOPPED);
    pros::delay(500);
    printToBrain(smallText, 25, 20, "Testing Pneumatics...        ");
    funnel_pneumatic_left.set_value(true);
    funnel_pneumatic_right.set_value(true);
    pros::delay(500);
    funnel_pneumatic_left.set_value(false);
    funnel_pneumatic_right.set_value(false);
    pros::delay(500);
    printToBrain(smallText, 25, 20, "Testing Drivetrain...      ");
    left_motors_drivetrain.move(60);
    right_motors_drivetrain.move(60);
    pros::delay(500);
    left_motors_drivetrain.move(0);
    right_motors_drivetrain.move(0);
    pros::delay(250);
    left_motors_drivetrain.move(-60);
    right_motors_drivetrain.move(-60);
    pros::delay(500);
    left_motors_drivetrain.move(0);
    right_motors_drivetrain.move(0);
    printToBrain(smallText, 25, 20, "Tests completed.      ");
    pros::delay(1000);
    return;
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

std::tuple startingSideValue = {1, -1};
std::string startingSide = "RED";
void autonomous() {

  // IMPORTANT
  // we need to create a system that can take in the starting side, and give the
  // correct path and outputs for auton mode
  int SS = 1; //default value
  if (startingSide == "BLUE") {
    int SS = std::get<0>(startingSideValue);
  } else {
    int SS = std::get<1>(startingSideValue);
  }
  // make sure we set the default position
  chassis.calibrate();
  chassis.setPose(0, 0, 0);
  // start auton period
  chassis.moveToPoint(0, 35, 3000);
  chassis.turnToHeading(SS*90, 1000);
  // move fowards and turn towards the gates

  // drive towards the gates
  chassis.moveToPoint(20, 35, 3000);
  // turn to face the loader
  chassis.turnToHeading(SS*180, 1000);
  // extend pneumatics to prepair for intaking
  funnel_pneumatic_left.extend();
  funnel_pneumatic_right.extend();
  // move into the loader
  chassis.moveToPoint(20, 10, 1000);
  chassis.setPose(20, 10, -180);
  // we are now in the loader
  //  we move back and fourth while intaking
  // to agitate the balls into going into our conveyor

  // start intaking balls
  updateBallConveyorMotors(UPPER_GOAL);
  // start agitation loop
  left_motors_drivetrain.move(80);
  right_motors_drivetrain.move(80);
  pros::delay(200);
  left_motors_drivetrain.move(-60);
  right_motors_drivetrain.move(-60);
  pros::delay(200);
  left_motors_drivetrain.move(80);
  right_motors_drivetrain.move(80);
  pros::delay(200);
  left_motors_drivetrain.move(-60);
  right_motors_drivetrain.move(-60);
  pros::delay(200);
  left_motors_drivetrain.move(80);
  right_motors_drivetrain.move(80);
  pros::delay(1200);
  left_motors_drivetrain.move(0);
  right_motors_drivetrain.move(0);
  // stop moving balls up after 2 seconds
  updateBallConveyorMotors(STOPPED);
  // now we need to drive backwards and score
  // after we go backwards and score in the high goals
  // we need to calibrate becuase the place where we are going to be scoring
  // can lock the robot into a known space
  // which means we can reset lemlib odometry to make it more accurate
  uint32_t time = pros::millis();
  if (upper_transit_motor.get_raw_position(&time) <
      /*expected posisiton*/ 1500) {
  }
  // get the posistion of the motors to ensure that they are moving correctly
}

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