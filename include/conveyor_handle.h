#pragma once

#include "main.h"

//create enum for changing the motor states
enum ball_conveyor_state {
  UPPER_GOAL,  // intake balls || spin both up
  MIDDLE_GOAL, // outtake balls || spin INTAKE_TRANSIST up, UPPER_TRANSIT down
  OUTTAKE,     // outtake balls || spin both down
  STOPPED      // stop both motors
};

// Declare the global state variable as 'extern'.
extern ball_conveyor_state current_ball_conveyor_state;
extern ball_conveyor_state new_state;
//
extern pros::Motor upper_transit_motor;
extern pros::Motor intake_transit_motor;

// foward declarations for the functions inside of the cpp file
void updateBallConveyorMotors(ball_conveyor_state new_state);
