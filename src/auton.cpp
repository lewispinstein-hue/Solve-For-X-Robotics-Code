#include "auton.h"
#include "conveyor_handle.h"
#include "setup.h"

struct WaitEvent {
  double x;               // X position to trigger
  double y;               // Y position (optional)
  int waitTimeMs;         // how long to wait
  double tolerance = 0.1; // relative positional error for event to trigger
  bool triggered = false; // triggered when the robot is at the event
};

std::vector<WaitEvent> waits = {
    // point for extending pneumatics
    {-47, 47, 200, 0.2},
    // event for inside of the loader
    {67.8, -47.5, 2000, 0.3},
    // event for scoring in the high goals
    {25.0, 46.0, 2000, 0.2}};

// PROVIDED BY https://path.jerryio.com
// Note: Asset file is static/jerryio_path1 (no .txt extension for symbol
// compatibility)

ButtonPressed sideColor;
ButtonPressed sideSide;
void selectRoute() {
  drawBottomButtons();
  printToBrain(smallText, 25, 20, "Pick path color");
  ButtonPressed sideColor = waitForBottomButtonTap();
  printToBrain(smallText, 25, 40, "Pick path side");
  ButtonPressed sideSide = waitForBottomButtonTap();
}
bool checkForEvents(int eventIndex) {
  // if the robot is within the tolerance of the event, set the event to
  // triggered and return true

  // checking the robots position against the wanted position
  if (fabs(chassis.getPose().x - waits[eventIndex].x) <
          waits[eventIndex].tolerance &&
      fabs(chassis.getPose().y - waits[eventIndex].y) <
          waits[eventIndex].tolerance) {
    waits[eventIndex].triggered = true;
  }
  // if the robot is not within the tolerance of the event, return false
  return waits[eventIndex].triggered;
}

ASSET(BlueLeftS1_txt); // moves from starting pos to inbetween loader and high
// goal
ASSET(BlueLeftS2_txt); // moves from endpoint of s1 into high goal

// path for starting on left side of red parking spot
void startingLeft() {
  chassis.setPose(64, -14, 270);
  // we move to the inbetween loader and high goal
  chassis.follow(BlueLeftS1_txt, 4, 7000, true, false);
  while (chassis.isInMotion()) {
    pros::delay(30);
  }
  // then move backwards into the high goal
  // chassis.follow(BlueLeftS2_txt, 9, 5000, false, false);
  // start moving pre-loaded ball into high goal
  setConveyorMotors(UPPER_GOAL);
  // we wait 2 seconds for ball to score before stopping the conveyor
  pros::delay(2000);
  // stopping conveyor motors
  setConveyorMotors(STOPPED);
  // we move on to next step
}

void startingRight() {
  // params
}

// function to get calling during comp
void autonomous() {
  // case for starting on left side of the parking zone
  if (sideColor == RIGHT && sideSide == RIGHT ||
      sideColor == LEFT && sideSide == RIGHT) {
    startingLeft();
  } else if (sideColor == RIGHT && sideSide == LEFT ||
             sideColor == LEFT && sideSide == LEFT) {
    startingLeft();
  } else {
    // run program that moves robot foward and then turns left
    chassis.moveToPoint(0, 20, 1000);
    chassis.turnToHeading(90, 500);
  }
}
