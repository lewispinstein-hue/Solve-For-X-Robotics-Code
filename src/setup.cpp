#include "setup.h"
#include <vector>

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

// creating clear screen function
void clearScreen() {
  // just trying everything to clear the screen.
  pros::screen::erase_rect(0, 0, 480, 272);
  pros::screen::fill_rect(0, 0, 480, 272);
  pros::screen::erase();
}