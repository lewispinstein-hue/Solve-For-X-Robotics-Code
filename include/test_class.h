#include "main.h"
#include <cmath>
#include <string>
#include <vector>

#define printToBrain pros::screen::print
#define smallText pros::E_TEXT_SMALL

template <typename Func> class Test {
private:
  std::vector<double> input1;
  std::vector<double> input2;
  std::vector<double> input3;
  std::vector<double> expectedOutputs;
  Func func;
  std::string test_name;
  double tolerance;

public:
  // Constructor
  Test(const std::vector<double> &in1, const std::vector<double> &in2,
       const std::vector<double> &in3, const std::vector<double> &expOut,
       const std::string &test_name, Func f, double tol = 0.1)
      : input1(in1), input2(in2), input3(in3), expectedOutputs(expOut), test_name(test_name),func(f),
        tolerance(tol) {}

  // Run all tests and optionally print results
  int runTestsArgs2(int delay, bool printResults) {

    clearScreen();
    printToBrain(smallText, 1, "Running %s test...", test_name);
    pros::delay(delay);
    clearScreen();
    int tests_passed = 0;
    size_t nTests = expectedOutputs.size();
    double result;
    for (size_t i = 0; i < nTests; i++) {

      double arg1 = i < input1.size() ? input1[i] : 0.0;
      double arg2 = i < input2.size() ? input2[i] : 0.0;
      result = func(arg1, arg2);

      // compare with expected output
      if (std::fabs(result - expectedOutputs[i]) < tolerance) {
        tests_passed++;
        if (printResults) {
          printToBrain(smallText, 25, (i * 20) + 20, "Test %d passed.", i);
        }
      } else if (printResults) {
        printToBrain(smallText, 25, (i * 20) + 20,
                     "Test %d Failed. Expected: %.2f, got %.2f",
                     expectedOutputs[i], result);
      }
    }

    if (printResults) {
      printToBrain(smallText, 25, 120, "Tests passed: %d/%d", tests_passed,
                   expectedOutputs.size());
      pros::delay(delay * 2.5);
    }
    return tests_passed;
  }

  int runTestsArgs3(int delay, bool printResults) {

    clearScreen();
    printToBrain(smallText, 1, "Running %s test...", test_name);
    pros::delay(delay);
    clearScreen();
    int tests_passed = 0;
    size_t nTests = expectedOutputs.size();
    double result;
    for (size_t i = 0; i < nTests; i++) {

      double arg1 = i < input1.size() ? input1[i] : 0.0;
      double arg2 = i < input2.size() ? input2[i] : 0.0;
      double arg3 = i < input3.size() ? input3[i] : 0.0;
      result = func(arg1, arg2, arg3);

      // compare with expected output
      if (std::fabs(result - expectedOutputs[i]) < tolerance) {
        tests_passed++;
        if (printResults) {
          printToBrain(smallText, 25, (i * 20) + 20, "Test %d passed.", i);
        }
      } else if (printResults) {
        printToBrain(smallText, 25, (i * 20) + 20,
                     "Test %d Failed. Expected: %.2f, got %.2f",
                     expectedOutputs[i], result);
      }
    }

    if (printResults) {
      printToBrain(smallText, 25, 120, "Tests passed: %d/%d", tests_passed,
                   expectedOutputs.size());
      pros::delay(delay * 2.5);
    }

    return tests_passed;
  }
};