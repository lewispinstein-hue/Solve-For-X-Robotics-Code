#include "conveyor_handle.h"

// conveyor motors
pros::Motor upper_transit_motor(7);
pros::Motor intake_transit_motor(-2);

// create the default value
ball_conveyor_state current_ball_conveyor_state = STOPPED;
ball_conveyor_state set_state = STOPPED;

// this is the function that takes in a wanted conveyor state, and then tells
// the motors to move with the wanted values
void setConveyorMotors(ball_conveyor_state set_state, int voltage) {
  // update the conveyor state to reflect what the motors are doing
  current_ball_conveyor_state = set_state;
  switch (set_state) {
  case UPPER_GOAL:
    upper_transit_motor.move(voltage);
    intake_transit_motor.move(voltage);
    break;
  case MIDDLE_GOAL:
    upper_transit_motor.move(-voltage);
    intake_transit_motor.move(voltage);
    break;
  case OUTTAKE:
    upper_transit_motor.move(-voltage);
    intake_transit_motor.move(-voltage);
    break;
  case STOPPED:
    upper_transit_motor.move(0);
    intake_transit_motor.move(0);
    break;
  }
}