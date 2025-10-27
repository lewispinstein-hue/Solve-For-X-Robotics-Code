#include "main.h"
#include "setup.h"

// PROVIDED BY https://path.jerryio.com
//
void autonomousRoute1() {
  std::tuple startingSideValue = {1, -1};
  std::string startingSide = "RED";

  // IMPORTANT
  // we need to create a system that can take in the starting side, and give the
  // correct path and outputs for auton mode
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