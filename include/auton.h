#pragma once

#include "setup.h"

// routes for starting on blue
void startingLeftBlue();
void startingRightBlue();
// routes for starting on red
void startingLeftRed();
void startingRightRed();
// default auton that is called during comp
void autonomous();
//extern variables for starting side
extern std::string startingColor;
extern std::string startingSide;