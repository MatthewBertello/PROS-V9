#pragma once

#ifndef autons_cpp
#define autons_cpp

#include "headers/autons.h"
#include "headers/autonomous.h"
#include "headers/initialize.h"
#include "headers/config.h"

enum driveCommands
{
    driveTo,
    driveThrough,
    turnToPoint,
    turnToAngle
};

// //autonomous routines
void autonomous1()
{
    autonRobotFunction.addCommands(
        base, driveTo, 0, 10,
        base, driveTo, 10, 10,
        base, driveTo, 10, 0,
        base, driveTo, 0, 0,
        base, driveThrough, 0, 10,
        base, turnToPoint, 0, 0,
        base, turnToAngle, 0
    );
    pros::delay(2000);
    autonRobotFunction.addCommands(
        base, turnToAngle, 180
    );
}
void autonomous2()
{
}
void autonomous3()
{
}
void autonomous4()
{
}
void autonomous5()
{
}
void autonomous6()
{
}
void autonomous7()
{
}
void autonomous8()
{
}
void autonomous9()
{
}
void autonomous10()
{
}
void autonomous11()
{
}
void autonomous12()
{
}
void autonomous13()
{
}
void autonomous14()
{
}
void autonomous15()
{
}
void autonomous16()
{
}

#endif
