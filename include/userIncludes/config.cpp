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


const int FRONTLEFTDRIVE = 0; // port 1
const int FRONTRIGHTDRIVE = 1; // port 2
const int BACKLEFTDRIVE = 2; // port 3
const int BACKRIGHTDRIVE = 3; // port 4
const int STRAFEWHEEL = 4; // port 5
const int INTAKEMOTOR = 5; // port 6
const int SHOOTER = 6; // port 7
const int LIFTMOTOR = 7; // port 8

const int DEFAULT_SLEW_RATE = 8;
const int SLEW_REFRESH_RATE = 10;
const int MAX_MOTOR_SPEED = 127;
const int MIN_MOTOR_SPEED = -127;


#endif
