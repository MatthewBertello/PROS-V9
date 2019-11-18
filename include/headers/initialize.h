#pragma once

#ifndef initialize_h
#define initialize_h

#include "headers/config.h"
#include "headers/driveMotor.h"
#include "headers/robotDrive.h"
#include "headers/robotFunction.h"
#include "pros/motors.hpp"

extern pros::Motor motorOne;
extern pros::Motor motorThree;
extern pros::Motor motorFour;
extern pros::Motor motorFive;
extern pros::Motor motorSix;
extern pros::Motor motorSeven;
extern pros::Motor motorEight;

extern driveMotor driveMotors[TOTAL_MOTORS];


extern robotDrive mainDrive;


extern pros::ADIGyro driveGyro;

extern robotFunction autonRobotFunction;

extern systems *systemsArray[NUMBER_OF_SYSTEMS];

extern pros::Task runRobotFunctionTask;
extern pros::Task autonomousInUserControlTask;

#endif
