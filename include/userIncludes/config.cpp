#pragma once

#ifndef config_cpp
#define config_cpp

#include "main.h"

const int FIRSTPORT = 1;
const int SECONDPORT = 2;
const int THIRDPORT = 3;
const int FOURTHPORT = 4;
const int FIFTHPORT = 5;
const int SIXTHPORT = 6;
const int SEVENTHPORT = 7;
const int EIGTHPORT = 8;


const int FRONTLEFTDRIVE /*FIRSTPORT*/ = 0;
const int FRONTRIGHTDRIVE /*SECONDPORT*/ = 1;
const int BACKLEFTDRIVE /*THIRDPORT*/ = 2;
const int BACKRIGHTDRIVE /*FOURTHPORT*/ = 3;
const int STRAFEWHEEL /*FIFTHPORT*/ = 4;
const int INTAKEMOTOR /*SIXTHPORT*/ = 5;
const int SHOOTER /*SEVENTHPORT*/ = 6;
const int LIFTMOTOR /*EIGTHPORT*/ = 7;

//driveMotor.h
const float DEFAULT_SLEW_RATE = 8;
const bool SHOULD_SLEW_BY_DEFAULT = false;

//filter.h
const int DEFAULT_EMA_FILTER_PERIOD = 10;
const int DEFAULT_DEMA_FILTER_PERIOD = 10;
const int MEDIAN_FILTER_DEFAULT_WIDTH = 10;

//pidController.h
const float DEFAULT_PID_P = .1;
const float DEFAULT_PID_I = .05;
const float DEFAULT_PID_D = .01;
const float DEFAULT_PID_C = 0;
const float DEFAULT_PID_INTEGRAL_LIMIT = 254;
const float DEFAULT_PID_LOWER_INTEGRAL_BAND = 0;
const float DEFAULT_PID_UPPER_INTEGRAL_BAND = 1000;
const bool DEFAULT_PID_RESET_INTEGRAL_AT_CROSS = true;
const bool DEFAULT_PID_CHANGE_CONSTANT_DIRECTION = true;

//robotDrive.h
const float ROBOT_DRIVE_DEFAULT_WHEEL_DIAMETER = 2.783;
const float ROBOT_DRIVE_DEFAULT_WHEEL_DISTANCE_FROM_CENTER = 8;
const float ROBOT_DRIVE_DEFAULT_WHEEL_TICKS_PER_ROTATION = 360;
const float ROBOT_DRIVE_DEFAULT_DISTANCE_THRESHOLD = 25;
const float ROBOT_DRIVE_DEFAULT_ANGLE_THRESHOLD = 50;

//lcdCode.cpp
const int MAX_AUTONOMOUS_MODES = 16;

//autonomous.cpp
const bool DEFAULT_AUTONOMOUS_MODE = 1;

//taskFunctions.cpp
const int SLEW_REFRESH_RATE = 10;
const int MAX_MOTOR_SPEED = 127;
const int MIN_MOTOR_SPEED = -127;
const int DRIVE_TRACKER_REFRESH_RATE = 1;

//opControl.cpp
const int OP_CONTROL_REFRESH_RATE = 5;


#endif
