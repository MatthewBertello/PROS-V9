#pragma once

#ifndef autons_cpp
#define autons_cpp

#define syscom autonRobotFunction.addSystemCommand
#define actcom autonRobotFunction.addSystemCommand
#define addcom autonRobotFunction.addCommand
#define stm isASystem
#define atn isAnAction

#include "headers/autons.h"
#include "headers/autonomous.h"
#include "headers/initialize.h"

void insertManualCommands()
{
  while(!autonRobotFunction.finishedReading())
  {
    pros::delay(5);
  }
  runningRunRobotFunctionTask = false;
}

void resumeAuton()
{
  autonRobotFunction.resetRobotFunction();
  runningRunRobotFunctionTask = true;
}

enum Actions
{
  pause,
  pauseAll,
  maxTime,
  end
};

enum driveCommands
{
  pid,
  sweepRight,
  sweepLeft,
  forwardRight,
  forwardLeft,
  driveStraight,
  gyroDriveStraight,
  gyroTurn,

  PIDDriveCommandMaxValue,

  speed,
  driveSpeed,
  turnSpeed,
  sweepRightSpeed,
  sweepLeftSpeed,
  forwardRightSpeed,
  forwardLeftSpeed,
  driveStraightSpeed,
  gyroDriveStraightSpeed,
  gyroTurnSpeed,

  driveCommandMaxValue,
};

enum currentSystems
{
  base
};

enum Commands
{
  unWritten,
  goToStart,
  isASystem,
  isAnAction,
};

// //autonomous routines
void autonomous1 ()
{
  syscom(base, gyroDriveStraight, 0, 2500);
  actcom(maxTime, base, 1000);
  addcom(atn, pause, base, stm, base, gyroTurn, 900);
  insertManualCommands();
  pros::delay(3000);
  resumeAuton();
  syscom(base, gyroTurn, 0);
}
void autonomous2 ()
{
}
void autonomous3 ()
{
}
void autonomous4 ()
{
}
void autonomous5 ()
{
}
void autonomous6 ()
{
}
void autonomous7 ()
{
}
void autonomous8 ()
{
}
void autonomous9 ()
{
}
void autonomous10 ()
{
}
void autonomous11 ()
{
}
void autonomous12 ()
{
}
void autonomous13 ()
{
}
void autonomous14 ()
{
}
void autonomous15 ()
{
}
void autonomous16 ()
{
}

#endif
