#pragma once

// headers for setup
#include "conveyor_handle.h"
#include "lemlib/chassis/chassis.hpp"
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

// for creating drivetrain motors
extern pros::MotorGroup left_motors_drivetrain;
extern pros::MotorGroup right_motors_drivetrain;
// creating controller
extern pros::Controller main_controller;
// creating pneumatics
extern bool funnel_engaged;
extern pros::adi::Pneumatics funnel_pneumatic_right;
extern pros::adi::Pneumatics funnel_pneumatic_left;

// lemlib construcion
extern const float TRACK_WIDTH;
extern pros::Imu imu;
// IME's in motors
extern lemlib::TrackingWheel leftVerticalTrackingWheel;
extern lemlib::TrackingWheel rightVerticalTrackingWheel;
// odom sensors
extern lemlib::OdomSensors odomSensors;
// settings for lemlib
extern lemlib::ControllerSettings lateralSettings;
extern lemlib::ControllerSettings angularSettings;
// drivetrain and chassis
extern lemlib::Drivetrain main_drivetrain;
extern lemlib::Chassis chassis;

// foward declarations for tests
double expo_joystick_foward(double, double);
double custom_clamp(double, double, double);
double handleArcadeControl(double &, double &, double);
// foward declarations for other
void clearScreen();
void handleSetupSelections();
void testPhysicals();
extern std::vector<bool> testsToRun;

// users people
extern Users lewis;
extern Users eli;
extern Users sanjith;
extern Users ian;