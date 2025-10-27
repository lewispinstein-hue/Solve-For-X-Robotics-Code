// this is the file for running tests. Add tests here
#include "setup.h"

// software tests - internal
void handleSoftwareTests() {
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
    pros::delay(20); // prevent unneed strain on CPU
  }
}

// hardware tests - external
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

// this is the section when we ask what setup tasks to run
std::vector<bool> testsToRun(2, false);
void handleSetupSelections() {
  while (true) {
    printToBrain(smallText, 25, 40, "Would you like to run Software tests?");
    if (main_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      printToBrain(smallText, 25, 40, "Selected: TRUE");
      testsToRun[0] = true;
      break;
    } else if (main_controller.get_digital_new_press(
                   pros::E_CONTROLLER_DIGITAL_X)) {
      printToBrain(smallText, 25, 40, "Selected: FALSE");
      testsToRun[0] = false;
      break;
    }
    pros::delay(20); // prevent unneed strain on CPU
  }
  // after asking for software tests, we move to asking for hardware tests
  while (true) {
    printToBrain(smallText, 25, 40, "Would you like to run Physical tests?");
    if (main_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      printToBrain(smallText, 25, 40, "Selected: TRUE");
      testsToRun[1] = true;
      break;
    } else if (main_controller.get_digital_new_press(
                   pros::E_CONTROLLER_DIGITAL_X)) {
      printToBrain(smallText, 25, 40, "Selected: FALSE");
      testsToRun[1] = false;
      break;
    }
    pros::delay(20); // prevent unneed strain on CPU
  }
  return;
}