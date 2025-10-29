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
// _asset currentPath = path1_NEW;

// path for starting on left side of red parking spot

bool checkForEvents(int eventIndex) {
  // if the robot is within the tolerance of the event, set the event to
  // triggered and return true
  if (fabs(chassis.getPose().x - waits[eventIndex].x) <
          waits[eventIndex].tolerance &&
      fabs(chassis.getPose().y - waits[eventIndex].y) <
          waits[eventIndex].tolerance) {
    waits[eventIndex].triggered = true;
    return true;
  }
  // if the robot is not within the tolerance of the event, return false
  waits[eventIndex].triggered = false;
  return false;
}

ASSET(BlueLeftS1_txt); // moves from starting pos to inbetween loader and high
                       // goal
ASSET(BlueLeftS2_txt); // moves from endpoint of s1 into high goal

void startingLeftBlue() {
  // set pose to the starting position
  chassis.setPose(64.479, -13.731, 270);
  //we move to the inbetween loader and high goal
  chassis.follow(BlueLeftS1_txt, 11, 7000, true);
  //then move backwards into the high goal
  chassis.follow(BlueLeftS2_txt, 15, 5000, false, false);
  //start moving pre-loaded ball into high goal
  setConveyorMotors(OUTTAKE);
  //we wait 2 seconds for ball to score before stopping the conveyor
  pros::delay(2000);
  //stopping conveyor motors
  setConveyorMotors(STOPPED);
  //we move on to next step
}


//creating an input field for choosing where the robot is starting
std::tuple<std::string, std::string> getStartingSide() {
  const std::string startingColor = "BLUE";
  const std::string startingSide = "LEFT";
  return std::make_tuple(startingColor, startingSide);
}

// function to get calling during comp
void autonomous() {
  // case for starting on left side of blue
  if (std::get<0>(getStartingSide()) == "BLUE" &&
      std::get<1>(getStartingSide()) == "LEFT") {
    startingLeftBlue();
  }
  // case for starting on right side of blue
  else if (std::get<0>(getStartingSide()) == "BLUE" &&
           std::get<1>(getStartingSide()) == "RIGHT") {
    startingRightBlue();
  }
  // case for starting on left side of red
  else if (std::get<0>(getStartingSide()) == "RED" &&
           std::get<1>(getStartingSide()) == "LEFT") {
    startingLeftRed();
  }
  // case for starting on right side of red
  else if (std::get<0>(getStartingSide()) == "RED" &&
           std::get<1>(getStartingSide()) == "RIGHT") {
    startingRightRed();
  } else {
    //run program that moves robot foward and then turns left
    chassis.moveToPoint(0, 20, 1000);
    chassis.turnToHeading(90, 500);
  }
}
