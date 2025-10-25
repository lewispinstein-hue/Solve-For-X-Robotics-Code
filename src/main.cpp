#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.hpp"
#include "pros/screen.hpp"

// my helper files
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

// intake and transit motors
pros::Motor upper_transit_motor(7);
pros::Motor intake_transit_motor(-2);

// Lemlib initialization
constexpr float HALF_TRACK = 7.5f / 2.0f;

pros::Imu imu(16); // the slot for our imu

// tracking wheels are the built in IME's in the motors
lemlib::TrackingWheel leftVerticalTrackingWheel(&left_motors_drivetrain,
                                                lemlib::Omniwheel::NEW_325,
                                                HALF_TRACK, 600);

lemlib::TrackingWheel rightVerticalTrackingWheel(&right_motors_drivetrain,
                                                 lemlib::Omniwheel::NEW_325,
                                                 -HALF_TRACK, 600);

// sensor init with the sensors we created above
lemlib::OdomSensors
    odomSensors(&leftVerticalTrackingWheel,  // vertical1
                &rightVerticalTrackingWheel, // vertical2
                nullptr, // horizontal1 (no horizontal tracking wheel)
                nullptr, // horizontal2
                &imu     // IMU
    );

// settings lemlib uses (need to be tweaked)
lemlib::ControllerSettings lateralSettings(12, 0, 2, // kP, kI, kD
                                           0,      // integral anti-windup range
                                           1, 100, // small error range, timeout
                                           3, 500, // large error range, timeout
                                           30      // max acceleration (slew)
);

lemlib::ControllerSettings angularSettings(6, 0, 8, 0, 1, 0, 0, 0, 30);

// creating drivedrain object te be used in chassis
lemlib::Drivetrain main_drivetrain(
    &left_motors_drivetrain,  // left motor group
    &right_motors_drivetrain, // right motor group
    7.5, // track width in inches (measure center-to-center of wheels)
    lemlib::Omniwheel::NEW_325, // 3.25" omni wheels
    900, // wheel RPM (green cartridge = 200, blue = 600, red = 100)
    2    // chase power (leave as 2 unless tuning)
);

// creating chassis for odometry and PID
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

// foward declaration for tests
double expo_joystick(double, double);
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

  // test for expo_joystick
  Test testExpoJoystick({1, 64, 63, 127}, {1, 1.2, 3.1, 2}, {},
                        {1.0, 55.8, 14.5, 127.0}, "Expo Joystck", expo_joystick,
                        TOLERANCE);
  tests_passed += testExpoJoystick.runTestsArgs2(500, true);
  // tesing custom clamp
  Test customClampTest({305, -102, 491, -230}, {200, -400, 506, -215},
                       {400, -200, 609, -102}, {305, -200, 506, -215},
                       "Custom Clamp", custom_clamp, TOLERANCE);
  tests_passed += customClampTest.runTestsArgs3(500, true);

  std::vector<double> leftYArcade = {127, -127, 127, 0, 0, 20, 50, 35};
  std::vector<double> rightXArcade = {0, 0, 127, 127, -127, 0, 50, 80};
  std::vector<double> testingScaleFactor = {2, 2, 2, 2, 2, 2, 2, 2};
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

  current_ball_conveyor_state = STOPPED; // initial state

  // handle tests
  //   handleTests();
  //   clearScreen();
}

// visit users_class.h for User setup explanation
Users eli("Eli     ", 25, 40, 1.8, 1.6, Users::ControlType::Arcade,
          pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_R1,
          pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_B);

Users lewis("Lewis", 25, 40, 1.9, 1.4, Users::ControlType::Arcade,
            pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_R1,
            pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_B);

Users ian("Ian", 20, 30, 2.1, 1.5, Users::ControlType::Arcade,
          pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_R1,
          pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_A);

Users sanjith("Sanjith", 25, 40, 2.1, 1.5, Users::ControlType::Arcade,
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

  // testing buttons for lemlib
  else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
    chassis.setPose(0, 0, 0);
    chassis.cancelAllMotions();
    main_controller.print(0, 0, "Pose reset to 0,0,0");
  }
  // else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
  //   // chassis.turnToHeading(chassis.getPose().theta + 90, 1000);
  //   double prevTheta = chassis.getPose().theta;
  //   while (true) {
  //     double targetTheta = chassis.getPose().theta + 90.0;
  //     double kP = 2.0;     // Proportional constant, will need tuning
  //     double error = 1000; // Initialize with a large error

  //     while (fabs(error) >
  //            1.0) { // Keep turning until error is less than 1 degree
  //       error = targetTheta - chassis.getPose().theta;

  //       // Calculate motor speed based on error
  //       double motorSpeed = error * kP;

  //       // Clamp the speed to prevent it from being too high or too low
  //       if (motorSpeed > 100)
  //         motorSpeed = 100;
  //       if (motorSpeed < -100)
  //         motorSpeed = -100;

  //       left_motors_drivetrain.move(motorSpeed);
  //       right_motors_drivetrain.move(-motorSpeed);
  //     }
  //   }
  // } else if (main_controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
  //   chassis.moveToPoint(0, 10, 1000);
  // }
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
double EXPONENT = 1.7;

int track_user = 1;
constexpr auto sizeOfUsers = 4;
void setActiveUser() {
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
  EXPONENT = Users::currentUser->getExponent();
  // we do not need to update certain members like the keybinds and the drive
  // types because there is nothing to update them to
}

double expo_joystick(double input, double EXPONENT) {
  // function to apply an exponential curve to the input.
  // is is applyed to the fowards and turn values in opcontrol()
  double normalized = (double)input / 127.0;       // normalize to [-1, 1]
  double curved = pow(fabs(normalized), EXPONENT); // apply exponential curve
  curved = curved * (normalized >= 0 ? 1 : -1);    // restore sign

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
    left_motor_voltage = expo_joystick(LEFT_Y_AXIS, EXPONENT);
    right_motor_voltage = expo_joystick(RIGHT_Y_AXIS, EXPONENT);
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
  printToBrain(smallText, 25, 100, "Heading: %.2f | Corrected Heading: %.1f   ",
               pose.theta, correctedHeading);
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

void autonomous() {
  uint32_t prevTime = 0; // initialize prevTime outside the loop

  while (true) {
    checkControllerButtonPress();
    // chassis.moveToPoint(0, 25, 5000);

    // Only start the reversal logic after the first movement is complete.

    if (prevTime == 0) {
      prevTime =
          pros::millis(); // Set prevTime the first time we enter this part
    }

    // Check if 1 second has passed since we started moving backwards.
    // pros::millis();  returns the time since the program started.
    if (pros::millis() - prevTime < 3000) {
      // spin motors backwards while the 1s timer is true
      left_motors_drivetrain.move(-100);
      right_motors_drivetrain.move(-100);
    } else {
      // 1 second has passed, so we can now reset the chassis position and stop
      // the backwards movement.
      left_motors_drivetrain.move(0);
      right_motors_drivetrain.move(0);
      chassis.setPose(0, 12, 0);

      // chassis.turnToHeading(90, 800);
      // chassis.moveToPoint(15, 25, 1000);

      break;
      // }
    }
  }
}
/**
 * Runs the operator control code. This function will be started in its own
 * task with the default priority and stack size whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the
 * operator control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart
 * the task, not resume it from where it left off.
 */

void opcontrol() {
  while (true) {
    // make sure that the Users::currentUser contains a valid pointer
    if (Users::currentUser == nullptr) {
      printToBrain(smallText, 5,
                   "ERROR. NULLPTR ON POINTER 'Users::currentUser'. STOPPING");
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

    double forward = expo_joystick(LEFT_Y_AXIS, EXPONENT);
    double turn = expo_joystick(RIGHT_X_AXIS, EXPONENT);
    double left_motor_voltage = forward + turn;
    double right_motor_voltage = forward - turn;

    checkControllerButtonPress(); // Check if any controller buttons are
                                  // pressed
    updateBallConveyorMotors();   // handle the enum that controlls the ball
                                  // conveyors
    setActiveUser();
    handleDriving(LEFT_Y_AXIS, RIGHT_X_AXIS, RIGHT_Y_AXIS, left_motor_voltage,
                  right_motor_voltage);

    left_motors_drivetrain.move(left_motor_voltage);
    right_motors_drivetrain.move(right_motor_voltage);

    printDebug(LEFT_Y_AXIS, RIGHT_X_AXIS, left_motor_voltage,
               right_motor_voltage);

    pros::delay(20); // keep update time set to keep cpu happy :)
  }
}
