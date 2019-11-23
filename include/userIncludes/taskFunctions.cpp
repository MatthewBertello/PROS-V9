#pragma once

#ifndef taskFunctions_cpp
#define TaskFunctions_cpp

#include "headers/opcontrol.h"
#include "headers/autonomous.h"
#include "headers/driveMotor.h"
#include "headers/initialize.h"

void slewMotors(void *param)
{
  uint32_t lastRun = pros::c::millis();
  int motorIndex = 0;
  driveMotor *x;

  while (true)
  {
    while (motorIndex < TOTAL_MOTORS)
    {
      x = &(driveMotors[motorIndex]);

      if (x->slewEnabled())
      {
        if (x->getCurrentSpeed() < x->getRequestedSpeed())
        {
          x->setCurrentSpeed(x->getCurrentSpeed() + x->getSlewRate());
          x->setCurrentSpeed((x->getCurrentSpeed() > x->getRequestedSpeed()) ? x->getRequestedSpeed() : x->getCurrentSpeed());
        }
        if (x->getCurrentSpeed() > x->getRequestedSpeed())
        {
          x->setCurrentSpeed(x->getCurrentSpeed() - x->getSlewRate());
          x->setCurrentSpeed((x->getCurrentSpeed() < x->getRequestedSpeed()) ? x->getRequestedSpeed() : x->getCurrentSpeed());
        }
        x->setCurrentSpeed((x->getCurrentSpeed() > MAX_MOTOR_SPEED) ? MAX_MOTOR_SPEED : x->getCurrentSpeed());
        x->setCurrentSpeed((x->getCurrentSpeed() < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : x->getCurrentSpeed());

        x->move(x->getCurrentSpeed());
      }
      else
      {
        x->move(x->getRequestedSpeed());
      }
      motorIndex++;
    }
    motorIndex = 0;
    pros::c::task_delay_until(&lastRun, SLEW_REFRESH_RATE);
    lastRun = pros::c::millis();
  }
}

void mainDrivePositionTrackerFn(void *param)
{
  // uint32_t lastRun = pros::c::millis();
  // while (true)
  // {
  //   lastRun = pros::c::millis();
  //   mainDrive.trackPosition();
  //   mainDrive.trackVelocity();
  //   pros::c::task_delay_until(&lastRun, DRIVE_TRACKER_REFRESH_RATE);
  // }
}
#endif