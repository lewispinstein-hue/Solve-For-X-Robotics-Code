#pragma once

#include "setup.h"

// routes for starting on blue
void startingLeft();
void startingRight();
// default auton that is called during comp
void autonomous();
//extern variables for starting side
extern std::string startingColor;
extern std::string startingSide;
//function to choose side. Runs a program on the brain screen
void selectRoute();