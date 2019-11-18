#pragma once

#ifndef taskFunctions_cpp
#define TaskFunctions_cpp

void autonomousInUserControlFn(void *param)
{
  while (true)
  {
    while (runningAutoninUserControl)
    {
      autonomous();
      runningAutoninUserControl = false;
      pros::delay(10);
    }
    pros::delay(10);
  }
}

void runRobotFunctionTaskFn(void *param)
{
  while (true)
  {
    while (pros::competition::is_autonomous || runningRunRobotFunctionTask)
    {
      autonRobotFunction.exectueRobotFunction();
      pros::delay(1);
    }

    while (!pros::competition::is_autonomous && !runningRunRobotFunctionTask)
    {
      autonRobotFunction.resetRobotFunction();
      pros::delay(10);
    }
  }
}

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

        x->getMotorPointer()->move(x->getCurrentSpeed());
      }
      else
      {
        x->getMotorPointer()->move(x->getRequestedSpeed());
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
  uint32_t lastRun = pros::c::millis();
  while (true)
  {
    lastRun = pros::c::millis();
    mainDrivePositionTracker.trackPosition();
    mainDrivePositionTracker.trackVelocity();
    pros::c::task_delay_until(&lastRun, 1);
  }
}
#endif