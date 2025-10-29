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

ASSET(main_test_txt);

void autonomousRoute2() {
  // set pose to the starting position
  chassis.setPose(64.479, -13.731, 270);

  // Follow the path with 10 second timeout
  // The original 1000ms timeout was too short for this path
  chassis.follow(main_test_txt, 9, 10000, true);

  while (chassis.isInMotion()) {
    // check what path
    //  if (currentPath.buf == path1_NEW.buf) {
    if (checkForEvents(1)) {
      // the condition is met, so we need to extend the pneumatics
      updateBallConveyorMotors(OUTTAKE);
      pros::delay(waits[1].waitTimeMs);
      updateBallConveyorMotors(STOPPED);
    }
    // if (checkForEvents(1)) {
    //   // the condition for inside the loader is met
    //   // run agitation code here
    // }
    // if (checkForEvents(2)) {
    //   // the condition for scoring in the high goals is met
    //   // run scoring code here
    // }
  }
}

void autonomousRoute1() {
  // outdated code.
  // keep it here for reference so we have a reference point for the new code
  std::tuple startingSideValue = {1, -1};
  std::string startingSide = "RED";

  // IMPORTANT
  // we need to create a system that can take in the starting side, and give
  // the correct path and outputs for auton mode
  int SS = 1; // default value
  if (startingSide == "BLUE") {
    SS = std::get<0>(startingSideValue);
  } else {
    SS = std::get<1>(startingSideValue);
  }
  // make sure we set the default position
  chassis.calibrate();
  chassis.setPose(0, 0, 0);
  // start auton period
  chassis.moveToPoint(0, 35, 3000);
  chassis.turnToHeading(SS * 90, 1000);
  // move fowards and turn towards the gates

  // drive towards the gates
  chassis.moveToPoint(20, 35, 3000);
  // turn to face the loader
  chassis.turnToHeading(SS * 180, 1000);
  // extend pneumatics to prepair for intaking
  funnel_pneumatic_left.extend();
  funnel_pneumatic_right.extend();
  // move into the loader
  chassis.moveToPoint(20, 10, 1000);
  chassis.setPose(20, 10, -180);
  // we are now in the loader
  //  we move back and fourth while intaking
  // to agitate the balls into going into our conveyor

  // start intaking balls
  updateBallConveyorMotors(UPPER_GOAL);
  // start agitation loop
  left_motors_drivetrain.move(80);
  right_motors_drivetrain.move(80);
  pros::delay(200);
  left_motors_drivetrain.move(-60);
  right_motors_drivetrain.move(-60);
  pros::delay(200);
  left_motors_drivetrain.move(80);
  right_motors_drivetrain.move(80);
  pros::delay(200);
  left_motors_drivetrain.move(-60);
  right_motors_drivetrain.move(-60);
  pros::delay(200);
  left_motors_drivetrain.move(80);
  right_motors_drivetrain.move(80);
  pros::delay(1200);
  left_motors_drivetrain.move(0);
  right_motors_drivetrain.move(0);
  // stop moving balls up after 2 seconds
  updateBallConveyorMotors(STOPPED);
  // now we need to drive backwards and score
  // after we go backwards and score in the high goals
  // we need to calibrate becuase the place where we are going to be scoring
  // can lock the robot into a known space
  // which means we can reset lemlib odometry to make it more accurate
  uint32_t time = pros::millis();
  if (upper_transit_motor.get_raw_position(&time) <
      /*expected posisiton*/ 1500) {
  }
  // get the posistion of the motors to ensure that they are moving correctly
}
