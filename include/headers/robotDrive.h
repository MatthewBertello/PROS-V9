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

class robotDrive : public systems
{

public:
  enum driveLayouts
  {
    STRAIGHT_DRIVE,
    HOLONOMIC,
    H_DRIVE
  };

  enum driveCommands
  {
    driveTo,
    driveThrough,
    turnToPoint,
    turnToAngle
  };

  driveLayouts driveType = STRAIGHT_DRIVE;

  driveMotor *leftMotors[TOTAL_MOTORS] = {};
  driveMotor *rightMotors[TOTAL_MOTORS] = {};
  driveMotor *strafeMotors[TOTAL_MOTORS] = {};

  pros::ADIGyro *driveGyro = nullptr;
  pros::ADIEncoder *leftEncoder = nullptr;
  pros::ADIEncoder *rightEncoder = nullptr;
  pros::ADIEncoder *strafeEncoder = nullptr;

  PIDController distancePID = PIDController(.1, .05, .01, 5, 254, 0, 635, true, true);
  PIDController turnPID = PIDController(.1, .025, .1, 10, 800, 0, 300, true, true);
  PIDController driveTurnPID = PIDController(.0025, .00, .0, 0, 1000, 0, 500, true, true);

  timer drivePIDTimer;

  bool useGyroInsteadOfFreeEncoders = false;

  float leftSpeed = 0;  // left drive speed
  float rightSpeed = 0; // right drive speed
  float strafeSpeed = 0;
  int driveCommand = -1;

  float targetX = 0;
  float targetY = 0;
  float targetAngle = 0;

  float deltaX = 0;
  float deltaY = 0;
  float direction = 0;
  float distance = 0;

  float leftDriveSensorLastReset = 0;
  float rightDriveSensorLastReset = 0;
  float strafeDriveSensorLastReset = 0;

  float leftWheelDiameter = 2.783;
  float rightWheelDiameter = 2.783;
  float strafeWheelDiameter = 2.783;

  float leftDistanceFromCenter = 8;
  float rightDistanceFromCenter = 8;
  float strafeDistanceFromCenter = 8;

  float leftTicksPerRotation = 360.0;
  float rightTicksPerRotation = 360.0;
  float strafeTicksPerRotation = 360.0;

  int distanceThreshold = 25;
  int angleThreshold = 50;

  float heading = 0;

  int addSystemCommands(int readPosition, std::vector<int> &commands)
  {
    int numberOfCommands = 0;
    switch (commands[readPosition])
    {
    case driveTo:

      numberOfCommands = 3;
      break;

    case driveThrough:
      numberOfCommands = 3;
      break;

    case turnToPoint:
      numberOfCommands = 3;
      break;

    case turnToAngle:
      numberOfCommands = 2;
      break;

    default:

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
    if (systemDone)
    {
      systemTimer.zeroTimer();
      drivePIDTimer.stopTimer();
      drivePIDTimer.zeroTimer();

      driveTurnPID.resetPID();
      distancePID.resetPID();
      turnPID.resetPID();

      driveTurnPID.setPIDTarget(0);
      distancePID.setPIDTarget(0);
      turnPID.setPIDTarget(0);

      systemMaxTime = 0;
      systemDone = false;

      switch (systemCommands[systemReadPos])
      {
      case driveTo:
        systemReadPos++;
        targetX = systemCommands[systemReadPos];
        systemReadPos++;
        targetY = systemCommands[systemReadPos];
        systemReadPos++;
        break;

      case driveThrough:
        systemReadPos++;
        targetX = systemCommands[systemReadPos];
        systemReadPos++;
        targetY = systemCommands[systemReadPos];
        systemReadPos++;
        break;

      case turnToPoint:
        systemReadPos++;
        targetX = systemCommands[systemReadPos];
        systemReadPos++;
        targetY = systemCommands[systemReadPos];
        systemReadPos++;
        break;

      case turnToAngle:
        systemReadPos++;
        targetAngle = systemCommands[systemReadPos];
        systemReadPos++;
        break;

      default:
        systemReadPos++;
        break;
      }
    }
    executeSystemFunction();
    if (systemReadPos >= systemCommands.size())
    {
      systemCompleted = true;
    }
    return systemCompleted;
  }

  void setDriveType(driveLayouts driveType)
  {
    this->driveType = driveType;
  }
  void addGyro(pros::ADIGyro *gyro)
  {
    this->driveGyro = gyro;
  }
  void addLeftEncoder(pros::ADIEncoder *encoder)
  {
    this->leftEncoder = encoder;
    resetLeftDriveSensor();
  }
  void addRightEncoder(pros::ADIEncoder *encoder)
  {
    this->rightEncoder = encoder;
    resetRightDriveSensor();
  }
  void addStrafeEncoder(pros::ADIEncoder *encoder)
  {
    this->strafeEncoder = encoder;
    resetStrafeDriveSensor();
  }
  float getCurrentAngle()
  {
    if ((leftEncoder != nullptr && rightEncoder != nullptr) && !useGyroInsteadOfFreeEncoders)
      return getAngleAsGyro();
    else if (driveGyro != nullptr)
      return this->driveGyro->get_value();
    else if (leftMotors[0] != nullptr && rightMotors[0] != nullptr)
      return getAngleAsGyro();
    else
      return 0;
  }
  bool addLeftMotor(driveMotor *newMotor)
  {
    for (int i = 0; i < 7; i++)
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
  bool addStrafeMotor(driveMotor *newMotor)
  {
    for (int i = 0; i < 7; i++)
    {
      if (leftMotors[i] == nullptr)
      {
        strafeMotors[i] = newMotor;
        return true;
      }
    }
    return false;
  }
  void resetLeftDriveSensor()
  {
    leftDriveSensorLastReset = getAbsoluteLeftDriveSensor();
  }

  void resetRightDriveSensor()
  {
    leftDriveSensorLastReset = getAbsoluteRightDriveSensor();
  }

  void resetStrafeDriveSensor()
  {
    strafeDriveSensorLastReset = getAbsoluteStrafeDriveSensor();
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
        return leftMotors[0]->getMotorPointer()->get_position();
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
        return rightMotors[0]->getMotorPointer()->get_position();
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
        return strafeMotors[0]->getMotorPointer()->get_position();
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
        return leftMotors[0]->getMotorPointer()->get_position() - leftDriveSensorLastReset;
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
      if (leftMotors[0] != nullptr)
        return rightMotors[0]->getMotorPointer()->get_position() - rightDriveSensorLastReset;
      else
        return 0;
    }
  }
  float getStrafeDriveSensor()
  {
    if (strafeEncoder != nullptr)
    {
      return strafeEncoder->get_value() - strafeDriveSensorLastReset;
    }
    else
    {
      if (leftMotors[0] != nullptr)
        return strafeMotors[0]->getMotorPointer()->get_position() - strafeDriveSensorLastReset;
      else
        return 0;
    }
  }

  void moveStraightDrive(int left, int right)
  {
    for (int i = 0; i < 7; i++)
    {
      if (leftMotors[i] != nullptr)
      {
        leftMotors[i]->setRequestedSpeed(left);
      }
    }
    for (int i = 0; i < 7; i++)
    {
      if (rightMotors[i] != nullptr)
      {
        rightMotors[i]->setRequestedSpeed(right);
      }
    }
  }

  void executeSystemFunction()
  {
    if (!this->systemDone) // if the drive is not done
    {
      switch (driveCommand)
      {
      case driveTo:

        deltaX = targetX - currentX;
        deltaY = targetY - currentY;
        direction = atan(deltaY / deltaX);
        distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

        leftSpeed = rightSpeed = distancePID.calculatePID(distance);
        leftSpeed += (fabs(leftSpeed) * driveTurnPID.calculatePID(direction - getCurrentAngle()));
        rightSpeed -= (fabs(rightSpeed) * driveTurnPID.calculatePID(direction - getCurrentAngle()));

        if (distance <= distanceThreshold)
        {
          drivePIDTimer.startTimer();
        }
        if (drivePIDTimer.currentTime() > 300)
          this->systemDone = true;
        break;

      case driveThrough:

        deltaX = targetX - currentX;
        deltaY = targetY - currentY;
        direction = atan(deltaY / deltaX);
        distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

        leftSpeed = rightSpeed = 127;

        leftSpeed += (fabs(leftSpeed) * driveTurnPID.calculatePID(direction - getCurrentAngle()));
        rightSpeed -= (fabs(rightSpeed) * driveTurnPID.calculatePID(direction - getCurrentAngle()));

        if (distance <= distanceThreshold)
        {
          drivePIDTimer.startTimer();
        }
        if (drivePIDTimer.currentTime() > 300)
          this->systemDone = true;
        break;

      case turnToPoint:
        deltaX = targetX - currentX;
        deltaY = targetY - currentY;
        direction = atan(deltaY / deltaX);

        leftSpeed = turnPID.calculatePID(direction - getCurrentAngle());
        rightSpeed = -leftSpeed;

        if (direction - getCurrentAngle() <= angleThreshold)
        {
          drivePIDTimer.startTimer();
        }
        if (drivePIDTimer.currentTime() > 300)
          this->systemDone = true;
        break;

      case turnToAngle:
        direction = targetAngle;

        leftSpeed = turnPID.calculatePID(direction - getCurrentAngle());
        rightSpeed = -leftSpeed;

        if (direction - getCurrentAngle() <= angleThreshold)
        {
          drivePIDTimer.startTimer();
        }
        if (drivePIDTimer.currentTime() > 300)
          this->systemDone = true;
        break;

      default:
        leftSpeed = 0;
        rightSpeed = 0;
      }

      moveStraightDrive(leftSpeed, rightSpeed);
    }
  }

  float currentAngle = 0;
  float currentX = 0;
  float currentY = 0;
  float previousLeft = 0;
  float previousRight = 0;
  float previousStrafe = 0;
  bool positionTrackingfirstRun = true;

  float angleVelocity = 0;
  float xVelocity = 0;
  float yVelocity = 0;
  float previousAngle = 0;
  float previousX = 0;
  float previousY = 0;

  float velocityRefresh = 100;

  uint32_t previousVelocityCheck = pros::c::millis();

  float getwheelDistanceMoved(float rotationsMoved, float wheelDiameter, float ticksPerRotation)
  {
    return wheelDiameter * PI / ticksPerRotation;
  }

  int getAngleAsGyro()
  {
    return (int)((currentAngle * (180 / PI)) * 10);
  }

  void setPosition(float x, float y, float angle)
  {
    currentAngle = angle;
    currentX = x;
    currentY = y;
  }

  void resetPosition()
  {
    positionTrackingfirstRun = true;
    currentAngle = 0;
    currentX = 0;
    currentY = 0;
  }

  void trackVelocity()
  {
    if (pros::c::millis() - previousVelocityCheck > velocityRefresh)
    {
      angleVelocity = ((currentAngle - previousAngle) * 1000) / previousVelocityCheck;
      xVelocity = ((currentX - previousX) * 1000) / previousVelocityCheck;
      yVelocity = ((currentY - previousY) * 1000) / previousVelocityCheck;

      previousAngle = currentAngle;
      previousX = currentX;
      previousY = currentY;
    }
  }

  void trackPosition()
  {
    float left = getAbsoluteLeftDriveSensor();
    float right = getAbsoluteRightDriveSensor();
    float strafe = getAbsoluteStrafeDriveSensor();

    float leftWheelDistance = getwheelDistanceMoved(left - previousLeft, leftWheelDiameter, leftTicksPerRotation);
    float rightWheelDistance = getwheelDistanceMoved(right - previousRight, rightWheelDiameter, rightTicksPerRotation);
    float strafeWheelDistance = getwheelDistanceMoved(strafe - previousStrafe, strafeWheelDiameter, strafeTicksPerRotation);

    previousLeft = left;
    previousRight = right;
    previousStrafe = strafe;

    if (positionTrackingfirstRun)
    {
      positionTrackingfirstRun = false;
      return;
    }

    float angleChange = (leftWheelDistance - rightWheelDistance) / (leftDistanceFromCenter + rightDistanceFromCenter);

    float distanceTraveled;
    float distanceTraveledStrafe;
    float halfAngleChange = angleChange / 2;

    if (angleChange == 0)
    {
      distanceTraveled = rightWheelDistance;
      distanceTraveledStrafe = strafeWheelDistance;
    }
    else
    {
      float rightRadius = rightWheelDistance / angleChange;
      float sinHalfAngleChange = sin(halfAngleChange);
      distanceTraveled = ((rightRadius + rightDistanceFromCenter) * sinHalfAngleChange) * 2.0;

      float strafeRadius = strafeWheelDistance / angleChange;
      distanceTraveledStrafe = ((strafeRadius + strafeDistanceFromCenter) * sinHalfAngleChange) * 2.0;
    }

    float cosAngleTraveled = cos(halfAngleChange + currentAngle);
    float sinAngleTraveled = sin(halfAngleChange + currentAngle);

    currentY += distanceTraveled * cosAngleTraveled;
    currentX += distanceTraveled * sinAngleTraveled;

    currentY += distanceTraveledStrafe * cosAngleTraveled;
    currentX += distanceTraveledStrafe * sinAngleTraveled;

    currentAngle += angleChange;
  }
};

#endif
