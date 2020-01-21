#pragma once

#ifndef autons_cpp
#define autons_cpp

#include "headers/autons.h"
#include "headers/autonomous.h"
#include "headers/initialize.h"
#include "headers/config.h"


enum driveCommands
{
  waitFor,
  maxTime,
  end,
  waitForTime,
  driveTo,
  driveThrough,
  turnToPoint,
  turnToAngle
};

// //autonomous routines
void autonomous1()
{
  driveMotors[LEFTINTAKE].setRequestedSpeed(-127);
  driveMotors[RIGHTINTAKE].setRequestedSpeed(-127);
  // driveMotors[RAMP].setRequestedSpeed(127);
  pros::delay(1000);

  driveMotors[LEFTINTAKE].setRequestedSpeed(0);
  driveMotors[RIGHTINTAKE].setRequestedSpeed(0);
  driveMotors[FRONTLEFTDRIVE].setRequestedSpeed(-127);
  driveMotors[FRONTRIGHTDRIVE].setRequestedSpeed(-127);
  driveMotors[BACKLEFTDRIVE].setRequestedSpeed(-127);
  driveMotors[BACKRIGHTDRIVE].setRequestedSpeed(-127);
  pros::delay(1000);
  driveMotors[FRONTLEFTDRIVE].setRequestedSpeed(127);
  driveMotors[FRONTRIGHTDRIVE].setRequestedSpeed(127);
  driveMotors[BACKLEFTDRIVE].setRequestedSpeed(127);
  driveMotors[BACKRIGHTDRIVE].setRequestedSpeed(127);
  pros::delay(400);
  driveMotors[FRONTLEFTDRIVE].setRequestedSpeed(0);
  driveMotors[FRONTRIGHTDRIVE].setRequestedSpeed(0);
  driveMotors[BACKLEFTDRIVE].setRequestedSpeed(0);
  driveMotors[BACKRIGHTDRIVE].setRequestedSpeed(0);
  // driveMotors[RAMP].setRequestedSpeed(-127);
  // pros::delay(300);
  // driveMotors[LEFTINTAKE].setRequestedSpeed(127);
  // driveMotors[RIGHTINTAKE].setRequestedSpeed(127);
  // autonRobotFunction.addCommands(
  //   base, driveTo, -10, 0
  // );
  // mainDrive.inversed = true;
  // autonRobotFunction.addCommands(
  //   base, turnToAngle, 24, 10,
  //   base, driveTo, 24, 10
  // );
  // mainDrive.inversed = false;
  // autonRobotFunction.addCommands(
  //   base, turnToAngle, 0,
  //   base, driveTo, 24, 48,
  //   base, turnToPoint, 44, 4,
  //   base, driveTo, 44, 4
  // );
  // driveMotors[LEFTINTAKE].setRequestedSpeed(0);
  // driveMotors[RIGHTINTAKE].setRequestedSpeed(0);
  //
  // driveMotors[RAMP].setRequestedSpeed(127);
  // pros::delay(500);
  // driveMotors[RAMP].setRequestedSpeed(0);
  // driveMotors[LEFTINTAKE].setRequestedSpeed(-127);
  // driveMotors[RIGHTINTAKE].setRequestedSpeed(-127);
  // pros::delay(250);
  // driveMotors[LEFTINTAKE].setRequestedSpeed(0);
  // driveMotors[RIGHTINTAKE].setRequestedSpeed(0);
  // autonRobotFunction.addCommands(
  //   base, driveTo, 0, 48
  // );
  // autonRobotFunction.addCommands(
  // base, driveTo, 0, 10
  // );
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
