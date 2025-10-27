// this is the file for running tests. Add tests here
#include "setup.h"

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