#include "setup.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/rotation.hpp"
#include "pros/screen.h"

// init drivetrain motor groups and controller.
// sets motor to blue gear cartridge, inits ports, and starts tracking the
// encoding for the motors in degrees
pros::MotorGroup left_motors_drivetrain({-6, 13, -14},
                                        pros::v5::MotorGears::rpm_600,
                                        pros::v5::MotorUnits::degrees);

pros::MotorGroup right_motors_drivetrain({8, 18, -20},
                                         pros::v5::MotorGears::rpm_600,
                                         pros::v5::MotorUnits::degrees);

// creating controller
pros::Controller main_controller(pros::E_CONTROLLER_MASTER);
// pneumatics
bool funnel_engaged = false;
pros::adi::Pneumatics funnel_pneumatic_right('H', funnel_engaged);
pros::adi::Pneumatics funnel_pneumatic_left('G', funnel_engaged);

// lemlib objects
//  Lemlib initialization

// the length of our robot from furthest wheel to closest wheel on the same side
constexpr float TRACK_WIDTH = 10.25;

pros::Imu imu(16); // the slot for our imu
// slot for horizontal1 tracking wheel
pros::Rotation horizontal1(15);
// tracking wheels are the built in IME's in the motors
lemlib::TrackingWheel leftVerticalTrackingWheel(&left_motors_drivetrain,
                                                lemlib::Omniwheel::NEW_325,
                                                (-TRACK_WIDTH / 2), 400);

lemlib::TrackingWheel rightVerticalTrackingWheel(&right_motors_drivetrain,
                                                 lemlib::Omniwheel::NEW_325,
                                                 (TRACK_WIDTH / 2), 400);
// external sensor for tracking horizontal movement
lemlib::TrackingWheel horizontalTrackingWheel(&horizontal1,
                                              lemlib::Omniwheel::NEW_2, 0.5);
// sensor init with the sensors we created above

lemlib::OdomSensors
    odomSensors(&leftVerticalTrackingWheel,  // vertical1
                &rightVerticalTrackingWheel, // vertical2
                &horizontalTrackingWheel,    // horizontal1
                nullptr, // horizontal2 (no horizontal tracking wheel)
                &imu     // IMU
    );

// settings lemlib uses (need to be tweaked)
lemlib::ControllerSettings lateralSettings(12, 0, 40, // kP, kI, kD
                                           0,    // integral anti-windup range
                                           0, 0, // small error range, timeout
                                           0, 0, // large error range, timeout
                                           0     // max acceleration (slew)
);

lemlib::ControllerSettings angularSettings(4, 0, 44, 0, 0, 0, 0, 0, 0);

// creating drivedrain object te be used in chassis
lemlib::Drivetrain main_drivetrain(
    &left_motors_drivetrain,  // left motor group
    &right_motors_drivetrain, // right motor group
    8.0625, // track width in inches (measure center-to-center of wheels)
    lemlib::Omniwheel::NEW_325, // 3.25" omni wheels
    600, // wheel RPM (green cartridge = 200, blue = 600, red = 100)
    2    // chase power (leave as 2 unless tuning)
);

// creating chassis for odometry and PID
lemlib::Chassis chassis(main_drivetrain, lateralSettings, angularSettings,
                        odomSensors);

// creating each user with their wanted settings
// visit users_class.h for User setup explanation
Users eli("Eli     ", 25, 40, 1.8, 3, 3, Users::ControlType::Arcade,
          pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_L2,
          pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_B);

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

// creating clear screen function
void clearScreen() {
  // just trying everything to clear the screen.
  pros::screen::erase_rect(0, 0, 480, 272);
  pros::screen::fill_rect(0, 0, 480, 272);
  pros::screen::erase();
}

// from here: code for checking if the screen is pressed
constexpr int SCREEN_WIDTH = 480;
constexpr int SCREEN_HEIGHT = 240;
constexpr int BUTTON_DIAMETER = 75; // circle diameter
constexpr int BUTTON_RADIUS = BUTTON_DIAMETER / 2;
constexpr int BUTTON_Y_CENTER =
    SCREEN_HEIGHT - BUTTON_RADIUS - 5; // a bit above bottom edge
constexpr int BUTTON_SPACING = 20;     // gap between circles

void drawBottomButtons() {
  clearScreen();

  // Calculate the centers for the three circles
  int center1_x = SCREEN_WIDTH / 6; // roughly 1/6 of width (left)
  int center2_x = SCREEN_WIDTH / 2; // middle
  int center3_x = SCREEN_WIDTH - SCREEN_WIDTH / 6; // right

  // LEFT circle
  pros::screen::set_pen(pros::c::COLOR_BLUE);
  pros::screen::fill_circle(center1_x, BUTTON_Y_CENTER, BUTTON_RADIUS);

  // MIDDLE circle
  pros::screen::set_pen(pros::c::COLOR_GREEN);
  pros::screen::fill_circle(center2_x, BUTTON_Y_CENTER, BUTTON_RADIUS);

  // RIGHT circle
  pros::screen::set_pen(pros::c::COLOR_RED);
  pros::screen::fill_circle(center3_x, BUTTON_Y_CENTER, BUTTON_RADIUS);

  // Label each circle
  pros::screen::set_pen(pros::c::COLOR_WHITE); 
  pros::screen::print(TEXT_MEDIUM, center1_x - 25, BUTTON_Y_CENTER - 10,
                      "LEFT");
  pros::screen::print(TEXT_MEDIUM, center2_x - 20, BUTTON_Y_CENTER - 10, "MID");
  pros::screen::print(TEXT_MEDIUM, center3_x - 30, BUTTON_Y_CENTER - 10,
                      "RIGHT");
}

/*
 Wait for a tap (press+release) inside the bottom button area.
 Uses release_count to detect the tap event which is compatible across PROS
 versions.
*/

constexpr int CENTER_LEFT = SCREEN_WIDTH / 6;
constexpr int CENTER_MIDDLE = SCREEN_WIDTH / 2;
constexpr int CENTER_RIGHT = SCREEN_WIDTH - SCREEN_WIDTH / 6;

ButtonPressed waitForBottomButtonTap() {
  int last_release_count = pros::screen::touch_status().release_count;

  while (true) {
    screen_touch_status_s touch = pros::screen::touch_status();

    // Detect a release event (finger lifted)
    if (touch.release_count != last_release_count) {
      last_release_count = touch.release_count;

      int x = touch.x;
      int y = touch.y;

      // Calculate squared distances to each circle
      int dxL = x - CENTER_LEFT;
      int dyL = y - BUTTON_Y_CENTER;
      int dxM = x - CENTER_MIDDLE;
      int dyM = y - BUTTON_Y_CENTER;
      int dxR = x - CENTER_RIGHT;
      int dyR = y - BUTTON_Y_CENTER;

      int r2 = pow(BUTTON_RADIUS, 2);

      // check if the button press is in the desired area
      if (pow(dxL, 2) + pow(dyL, 2) <= r2)
        return LEFT;
      if (pow(dxM, 2) + pow(dyM, 2) <= r2)
        return MIDDLE;
      if (pow(dxR, 2) + pow(dyR, 2) <= r2)
        return RIGHT;
    }

    pros::delay(20);
  }
}
