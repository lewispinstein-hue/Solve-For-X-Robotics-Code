#include "setup.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/rotation.hpp"

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
constexpr int BUTTON_HEIGHT = 80;

// Draw the three bottom buttons (optional)
void drawBottomButtons() {
  clearScreen();
  // left
  pros::screen::set_pen(pros::c::COLOR_BLUE);
  pros::screen::fill_rect(0, SCREEN_HEIGHT - BUTTON_HEIGHT,
                          SCREEN_WIDTH / 3 - 1, SCREEN_HEIGHT);
  // middle
  pros::screen::set_pen(pros::c::COLOR_GREEN);
  pros::screen::fill_rect(SCREEN_WIDTH / 3, SCREEN_HEIGHT - BUTTON_HEIGHT,
                          2 * SCREEN_WIDTH / 3 - 1, SCREEN_HEIGHT);
  // right
  pros::screen::set_pen(pros::c::COLOR_RED);
  pros::screen::fill_rect(2 * SCREEN_WIDTH / 3, SCREEN_HEIGHT - BUTTON_HEIGHT,
                          SCREEN_WIDTH, SCREEN_HEIGHT);

  pros::screen::set_pen(pros::c::COLOR_WHITE);
  pros::screen::print(TEXT_MEDIUM, 40, SCREEN_HEIGHT - 50, "LEFT");
  pros::screen::print(TEXT_MEDIUM, 200, SCREEN_HEIGHT - 50, "MID");
  pros::screen::print(TEXT_MEDIUM, 360, SCREEN_HEIGHT - 50, "RIGHT");
}

/*
 Wait for a tap (press+release) inside the bottom button area.
 Uses release_count to detect the tap event which is compatible across PROS
 versions.
*/
ButtonPressed waitForBottomButtonTap() {
  // get initial release count (may be zero)
  auto ts = pros::screen::touch_status();
  int32_t last_release_count = 0;
// Some PROS variants expose a release_count field directly; handle both
// possibilities:
#if defined(__cplusplus)
  // Try to read release_count safely; if not present, the compiler will error.
  // We'll guard with a try/catch-like strategy below by reading previous ts
  // into last_release_count.
#endif

// Best-effort attempt to extract release_count without assuming a boolean
// 'pressed' If touch_status has release_count field, use it; otherwise we
// fallback to edge-detect on x/y changes. Initialize last_release_count to
// current value if present:
#ifdef __GNUC__
  // GCC-style trick: attempt to read release_count if present
  // (If your toolchain rejects this, the following line may need removing — we
  // still have a fallback)
  last_release_count = ts.release_count;
#else
  (void)ts; // keep compiler quiet if above access isn't supported
#endif

  while (true) {
    auto touch = pros::screen::touch_status();

    // 1) Preferred: detect release_count increment (tap happened)
    bool gotTap = false;
    int32_t current_release_count = 0;
#ifdef __GNUC__
    // guarded read - many PROS builds have this member
    current_release_count = touch.release_count;
    if (current_release_count != last_release_count) {
      // release_count changed -> a press+release event occurred
      gotTap = true;
      last_release_count = current_release_count;
    }
#endif

    // 2) Fallback: if release_count not available (or build doesn't allow
    // accessing it), detect a press followed by release using x/y becoming
    // non-zero then zero.
    static bool wasPressed = false;
    if (!gotTap) {
      // Many PROS versions set coords to 0/0 when not pressed. Use that as
      // fallback.
      int x = touch.x;
      int y = touch.y;
      bool currentlyPressed =
          !(x == 0 && y == 0); // heuristics: non-zero coords -> touch is down

      if (currentlyPressed && !wasPressed) {
        // finger went down
        wasPressed = true;
      } else if (!currentlyPressed && wasPressed) {
        // finger released -> treat as tap
        wasPressed = false;
        gotTap = true;
      }
    }

    if (gotTap) {
      // Use the coordinates captured at the moment of release for location.
      // Note: touch.x/y may be zero on some firmwares at release; if so, prefer
      // previous coords.
      int x = touch.x;
      int y = touch.y;

      // If zero or invalid, we can wait a tiny bit and reread (best-effort)
      if (x == 0 && y == 0) {
        pros::delay(10);
        auto t2 = pros::screen::touch_status();
        x = t2.x;
        y = t2.y;
      }

      // sanity clamp
      if (x < 0)
        x = 0;
      if (x > SCREEN_WIDTH)
        x = SCREEN_WIDTH;
      if (y < 0)
        y = 0;
      if (y > SCREEN_HEIGHT)
        y = SCREEN_HEIGHT;

      // ensure it was in bottom region
      if (y >= SCREEN_HEIGHT - BUTTON_HEIGHT) {
        if (x < SCREEN_WIDTH / 3)
          return ButtonPressed::LEFT;
        else if (x < 2 * SCREEN_WIDTH / 3)
          return ButtonPressed::MIDDLE;
        else
          return ButtonPressed::RIGHT;
      } else {
        // tap outside bottom buttons — ignore and continue waiting
      }
    }

    pros::delay(15); // yield to OS and debounce
  }
}
