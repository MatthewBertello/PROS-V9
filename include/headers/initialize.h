#pragma once

#ifndef initialize_h
#define initialize_h

#include "headers/config.h"
#include "headers/driveMotor.h"
#include "headers/robotDrive.h"
#include "headers/robotFunction.h"
#include "pros/motors.hpp"

extern driveMotor motorOne;
extern driveMotor motorThree;
extern driveMotor motorFour;
extern driveMotor motorFive;
extern driveMotor motorSix;
extern driveMotor motorSeven;
extern driveMotor motorEight;

extern pros::ADIEncoder leftEncoder;
extern pros::ADIEncoder rightEncoder;
extern pros::ADIEncoder strafeEncoder;

extern driveMotor driveMotors[TOTAL_MOTORS];


extern robotDrive mainDrive;


extern pros::ADIGyro driveGyro;

extern robotFunction autonRobotFunction;

extern systems *systemsArray[NUMBER_OF_SYSTEMS];

extern pros::Task runRobotFunctionTask;
extern pros::Task autonomousInUserControlTask;

#endif
