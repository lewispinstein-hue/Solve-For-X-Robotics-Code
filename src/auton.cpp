#include "auton.h"
#include "conveyor_handle.h"
#include "main.h"
#include "pros/screen.h"
#include "setup.h"

// PROVIDED BY https://path.jerryio.com
// Note: Asset file is static/jerryio_path1 (no .txt extension for symbol
// compatibility)

ButtonPressed sideColor;
ButtonPressed sideSide;
void selectRoute() {
  drawBottomButtons();
  while (true) {
    printToBrain(smallText, 25, 20, "Pick path color");
    sideColor = waitForBottomButtonTap();
    if (sideColor != MIDDLE) {
      break; // Exit the loop if the input is valid
    }
    printToBrain(smallText, 25, 40, "Please pick a blue or red circle");
    continue;
  }
  while (true) {
    printToBrain(smallText, 25, 60, "Pick path side");
    sideSide = waitForBottomButtonTap();
    if (sideColor != MIDDLE) {
      break; // Exit the loop if the input is valid
    }
    printToBrain(smallText, 25, 80, "Please pick left or right");
    continue;
  }
  clearScreen();
  printToBrain(TEXT_MEDIUM, 25, 120, "Color: %s | Side: %s",
               (sideColor == 0) ? "Blue" : "Red",
               (sideSide == 0) ? "Left" : "Right");
  pros::delay(4000);
  clearScreen();
  return;
}

ASSET(BlueLeftS1_txt) // moves from starting pos to inbetween loader and high
// goal

// path for starting on left side of red parking spot
void startingLeft() {
  chassis.setPose(64, -14, 270);
  // we move to the inbetween loader and high goal
  chassis.follow(BlueLeftS1_txt, 10, 2000, true, true);
  while (chassis.isInMotion()) {
    // block actions
    pros::delay(20);
  }
  // make sure we are facing the exact right position
  chassis.moveToPoint(52, -47, 500, {}, false);
  chassis.turnToHeading(90, 1500, {}, false);
  // then move backwards into the high goal
  chassis.moveToPoint(22, -47, 1000, {.forwards = false, .maxSpeed = 60},
                      false);
  chassis.setPose(24, -47, 90);
  // start moving pre-loaded ball into high goal
  setConveyorMotors(UPPER_GOAL);
  // we wait 2 seconds for ball to score before stopping the conveyor
  pros::delay(2000);
  // stopping conveyor motors
  setConveyorMotors(STOPPED);
  // we move on to next step
  // move back to inbetween the goal and loader
  chassis.moveToPoint(42, -47, 600, {}, false);
  // move to balls
  chassis.moveToPoint(43, -22, 1000, {}, false);
  // face balls
  chassis.turnToHeading(270, 300);
  // next step: move to balls slowly and intake
}

ASSET(BlueRightT_txt)

void startingRight() {
  // set starting pose
  chassis.calibrate();
  chassis.setPose(64, 14, 270);
  // start route
  chassis.moveToPoint(46, 18, 1500, {.maxSpeed = 80}, false);
  //move to balls
  chassis.moveToPoint(39, 22, 1300, {.maxSpeed = 80}, false);
  //face balls
  chassis.turnToHeading(270, 400, {.maxSpeed = 80}, false);
  chassis.moveToPoint(39, 22, 900, {.maxSpeed = 80}, false);
  //move to pick up balls
  setConveyorMotors(UPPER_GOAL, 100);
  chassis.moveToPoint(16, 22, 1200, {.maxSpeed = 40}, false);
  setConveyorMotors( STOPPED);
  //turn to inbetween goal and loader
  chassis.turnToHeading(45, 500, {.maxSpeed = 80});
  //go to inbetween goal and loader
  chassis.moveToPoint(48, 47, 1500, {.maxSpeed = 80});
  // turn to correct heading
  chassis.turnToHeading(90, 800, {.maxSpeed = 80});
  chassis.waitUntilDone();
  chassis.moveToPoint(48, 47, 500, {.maxSpeed = 80});

  // move to goal
  chassis.turnToHeading(90, 1000, {.maxSpeed = 80});
  chassis.moveToPoint(20, 47, 2500, {.forwards = false, .maxSpeed = 40}, false);
  chassis.setPose(27, 47, 90);

  // reset pos to couteract motor slipping
  // start scoring ball
  setConveyorMotors(UPPER_GOAL);
  // we wait 2 seconds for ball to score before stopping the conveyor
  pros::delay(2000);
  // stopping conveyor motors
  setConveyorMotors(STOPPED);
  // moving to inbetween loader and goal
  chassis.moveToPoint(54, 47, 800, {.maxSpeed = 80});
  // turning to face loader
  chassis.turnToHeading(180, 1000);
  //go into parking zone:
  // chassis.moveToPoint(64, 14, 4000, {.maxSpeed = 60});
  // chassis.turnToHeading(270, 800);
}


// function to get calling during comp
void autonomous() {
  // case for starting on left side of the parking zone
  if (sideColor == RIGHT && sideSide == RIGHT ||
      sideColor == LEFT && sideSide == RIGHT) {
    startingRight();
  }
  // case for starting on the right side of the parking zone
  else if (sideColor == RIGHT && sideSide == LEFT ||
           sideColor == LEFT && sideSide == LEFT) {
    startingLeft();
  } else {
    // run program that moves robot foward and then turns left
    startingRight();
  }
}
