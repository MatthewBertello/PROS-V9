/*
#pragma once
#ifndef robotDrive_h
#define robotDrive_h

#include "headers/config.h"
#include "headers/systems.h"
#include <cmath>
#include "headers/mathFunctions.h"
#include "headers/gyroFunctions.h"
#include "headers/driveMotor.h"
#include "pros/adi.hpp"
#include "headers/timer.h"
#include "headers/pidController.h"
#include "pros/llemu.hpp"

extern systems *systemsArray[NUMBER_OF_SYSTEMS];

class robotDrive : public systems
{

public:

  enum liftCommands
  {
    waitFor,
    maxTime,
    end,
    waitForTime
  };

  driveMotor *leftMotors[TOTAL_MOTORS] = {};
  driveMotor *rightMotors[TOTAL_MOTORS] = {};

  int liftCommand = -1;

  pros::ADIGyro *driveGyro = nullptr;
  pros::ADIEncoder *leftEncoder = nullptr;

  PIDController distancePID = PIDController(12.7, .0, .0, 5, 0, 0, 635, true, true);

  timer drivePIDTimer;

  float leftSpeed = 0;  // left drive speed
  bool waitingForTimer = false;

  int addSystemCommands(int readPosition, std::vector<int> &commands)
  {
    int numberOfCommands = 0;
    switch (commands[readPosition])
    {

    case waitFor:
      numberOfCommands = 0;
      systemCommands.push_back(commands[readPosition]);
      systemCommands.push_back(commands[readPosition] + 1);
      systemCommands.push_back(systemsArray[commands[readPosition + 1]]->systemCommands.size());
      readPosition += 3;
      break;
    case maxTime:
      numberOfCommands = 2;
      break;
    case end:
      numberOfCommands = 2;
      break;
    case waitForTime:
      numberOfCommands = 2;
      break;
    default:
      numberOfCommands = 0;
      readPosition++;
      break;
    }
    for (int i = 0; i < numberOfCommands; i++)
    {
      systemCommands.push_back(commands[readPosition]);
      readPosition++;
    }
    return readPosition;
  }

  bool updateSystem()
  {

    if (systemReadPos < systemCommands.size())
    {
      switch (systemCommands[systemReadPos])
      {
      case waitFor:
      {
        if ((systemsArray[systemCommands[systemReadPos + 1]]->systemReadPos > systemCommands[systemReadPos] + 2) && systemsArray[systemCommands[systemReadPos + 1]]->systemDone)
        {
          systemReadPos += 3;
        }
      }
      break;
      case maxTime:
        systemMaxTime = systemCommands[systemReadPos + 1];
        systemReadPos += 2;
        break;
      case end:
        systemDone = true;
        systemMaxTime = 0;
        systemReadPos++;
        break;
      case waitForTime:
        if (systemDone)
        {
          if (systemMaxTime == 0)
          {
            systemTimer.zeroTimer();
            systemTimer.startTimer();
            systemMaxTime = systemCommands[systemReadPos + 1];
          }
          else if (systemTimer.currentTime() >= systemMaxTime)
          {
            systemReadPos += 2;
          }
        }
        break;
      default:
        if (systemDone)
        {
          systemTimer.stopTimer();
          systemTimer.zeroTimer();

          systemMaxTime = 0;
          systemDone = false;

          switch (systemCommands[systemReadPos])
          {
          case driveTo:
            driveCommand = driveTo;
            systemReadPos++;
            targetX = systemCommands[systemReadPos];
            systemReadPos++;
            targetY = systemCommands[systemReadPos];
            systemReadPos++;
            break;

          default:
            systemReadPos++;
            break;
          }
        }

        break;
      }
    }
    executeSystemFunction();
    if (systemReadPos >= systemCommands.size() && systemDone)
    {
      systemCompleted = true;
    }
    return systemCompleted;
  }

  void executeSystemFunction()
  {
    if (systemMaxTime != 0 && systemTimer.currentTime() >= systemMaxTime)
    {
      systemMaxTime = 0;
      systemDone = true;
    }
    if (!this->systemDone) // if the drive is not done
    {
      switch (liftCommand)
      {
      case driveTo:

        deltaX = targetX - currentX;
        deltaY = targetY - currentY;
        direction = atan(deltaY / deltaX);
        distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

        leftSpeed = rightSpeed = distancePID.calculatePID(-distance);
        leftSpeed += (fabs(leftSpeed) * driveTurnPID.calculatePID(gyroDirection(direction, getCurrentAngle()) * gyroDifference(direction, getCurrentAngle())));
        rightSpeed -= (fabs(rightSpeed) * driveTurnPID.calculatePID(gyroDirection(direction, getCurrentAngle()) * gyroDifference(direction, getCurrentAngle())));

        if (distance <= distanceThreshold)
        {
          drivePIDTimer.startTimer();
        }
        if (drivePIDTimer.currentTime() > 300)
        {
          leftSpeed = 0;
          rightSpeed = 0;
          this->systemDone = true;
        }
        break;

      default:
        leftSpeed = 0;
        rightSpeed = 0;
      }
      moveDrive(leftSpeed, rightSpeed);
    }
    else
    {
      systemMaxTime = 0;
      moveDrive(0, 0);
    }
  }
  bool addLeftMotor(driveMotor *newMotor)
  {
    for (int i = 0; i < TOTAL_MOTORS; i++)
    {
      if (leftMotors[i] == nullptr)
      {
        leftMotors[i] = newMotor;
        return true;
      }
    }
    return false;
  }
  bool addRightMotor(driveMotor *newMotor)
  {
    for (int i = 0; i < 7; i++)
    {
      if (rightMotors[i] == nullptr)
      {
        rightMotors[i] = newMotor;
        return true;
      }
    }
    return false;
  }

  float getAbsoluteLeftDriveSensor()
  {
    if (leftEncoder != nullptr)
    {
      return leftEncoder->get_value();
    }
    else
    {
      if (leftMotors[0] != nullptr)
        return leftMotors[0]->get_position();
      else
        return 0;
    }
  }
  float getAbsoluteRightDriveSensor()
  {
    if (rightEncoder != nullptr)
    {
      return rightEncoder->get_value();
    }
    else
    {
      if (rightMotors[0] != nullptr)
        return rightMotors[0]->get_position();
      else
        return 0;
    }
  }
  float getAbsoluteStrafeDriveSensor()
  {
    if (strafeEncoder != nullptr)
    {
      return strafeEncoder->get_value();
    }
    else
    {
      if (strafeMotors[0] != nullptr)
        return strafeMotors[0]->get_position();
      else
        return 0;
    }
  }
  float getLeftDriveSensor()
  {
    if (leftEncoder != nullptr)
    {
      return leftEncoder->get_value() - leftDriveSensorLastReset;
    }
    else
    {
      if (leftMotors[0] != nullptr)
        return leftMotors[0]->get_position() - leftDriveSensorLastReset;
      else
        return 0;
    }
  }
  float getRightDriveSensor()
  {
    if (rightEncoder != nullptr)
    {
      return rightEncoder->get_value() - rightDriveSensorLastReset;
    }
    else
    {
      if (rightMotors[0] != nullptr)
        return rightMotors[0]->get_position() - rightDriveSensorLastReset;
      else
        return 0;
    }
  }
};

#endif
*/
