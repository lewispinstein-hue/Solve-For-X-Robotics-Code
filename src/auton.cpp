#include "setup.h"

struct WaitEvent {
    double x;              // X position to trigger
    double y;              // Y position (optional)
    int waitTimeMs;        // how long to wait
    bool triggered = false;
};

std::vector<WaitEvent> waits = {
  //event for 
    {-66.0, 46.0, 1000},
    {36.0, 0.0, 500}
};

// PROVIDED BY https://path.jerryio.com
// Note: Asset file is static/jerryio_path1 (no .txt extension for symbol compatibility)

ASSET(jerryio_path1); 
//path for starting on left side of red parking spot

void autonomousRoute1() {
  chassis.setPose(-65.842, 13.889, 100);
  chassis.follow(jerryio_path1,9, 10000, true, true); // async

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


/*
for you gpt:
I want to know why this is not compiling
the error is:
David@MacBook-Air-5 Solve For X Robotics Code % pros make all
Cleaning project
Creating cold package with LemLib,libc,liblvgl,libm,libpros [OK]
Stripping cold package  [DONE]
Section sizes:
   text    data     bss   total     hex filename
2316309    6542 60017181        62340032        3b73bc0 bin/cold.package.elf
Compiled src/auton.cpp [OK]
Compiled src/conveyor_handle.cpp [OK]
Compiled src/main.cpp [OK]
Compiled src/setup.cpp [OK]
Compiled src/tests.cpp [OK]
Adding timestamp [OK]
Linking hot project with ./bin/cold.package.elf and LemLib,libc,liblvgl,libm,libpros [ERRORS]
/Users/David/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-macos/bin/../lib/gcc/arm-none-eabi/13.3.1/../../../../arm-none-eabi/bin/ld: bin/auton.cpp.o:/Users/David/Documents/VEXcode Robot/Solve For X Robotics Code/src/auton.cpp:18:(.data._ZL13jerryio_path1+0x0): undefined reference to `_binary_static_jerryio_path1_start'
/Users/David/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-macos/bin/../lib/gcc/arm-none-eabi/13.3.1/../../../../arm-none-eabi/bin/ld: bin/auton.cpp.o:(.data._ZL13jerryio_path1+0x4): undefined reference to `_binary_static_jerryio_path1_size'
collect2: error: ld returned 1 exit status
make: *** [bin/hot.package.elf] Error 1
ERROR - pros.cli.build:make - Failed to make project: Exit Code 2 - pros-cli version:3.5.6
PROS-CLI Version:  3.5.6
PROS-Kernel Version: 4.2.1
╭─ Error ──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────╮
│ Failed to build                                                                                                                  │
╰──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────╯
                                                                                                                                    
David@MacBook-Air-5 Solve For X Robotics Code % 
*/