#pragma once
#ifndef opcontrol_h
#define opcontrol_h

#include "headers/config.h"
#include "pros/misc.hpp"

extern bool curve;// should the robot use the parametric drive formula
extern bool inversed;// should the drive be inversed
extern bool control;// should the drive be in control mode

void singleControllerDrive();
void doubleControllerDrive();
void userAutonomous(void* param);

extern pros::Controller master;
extern pros::Controller partner;

#endif
