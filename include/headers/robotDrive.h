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
  enum driveLayouts
  {
    STRAIGHT_DRIVE,
    HOLONOMIC,
    H_DRIVE,
    TRIANGLE_DRIVE
  };

  enum driveCommands
  {
    waitFor,
    maxTime,
    end,
    waitForTime,
    driveTo,
    driveThrough,
    turnToPoint,
    turnToAngle,
  };

  driveLayouts driveType = STRAIGHT_DRIVE;

  driveMotor *leftMotors[TOTAL_MOTORS] = {};
  driveMotor *rightMotors[TOTAL_MOTORS] = {};
  driveMotor *centerMotors[TOTAL_MOTORS] = {};
  driveMotor *strafeMotors[TOTAL_MOTORS] = {};

  pros::ADIGyro *driveGyro = nullptr;
  pros::ADIEncoder *leftEncoder = nullptr;
  pros::ADIEncoder *rightEncoder = nullptr;
  pros::ADIEncoder *strafeEncoder = nullptr;

  PIDController distancePID = PIDController(12.7, .0, .0, 5, 0, 0, 635, true, true);
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

  float leftWheelDiameter = 3.45;
  float rightWheelDiameter = 3.45;
  float strafeWheelDiameter = ROBOT_DRIVE_DEFAULT_WHEEL_DIAMETER;

  float leftDistanceFromCenter = ROBOT_DRIVE_DEFAULT_WHEEL_DISTANCE_FROM_CENTER;
  float rightDistanceFromCenter = ROBOT_DRIVE_DEFAULT_WHEEL_DISTANCE_FROM_CENTER;
  float strafeDistanceFromCenter = 0;

  float leftTicksPerRotation = ROBOT_DRIVE_DEFAULT_WHEEL_TICKS_PER_ROTATION;
  float rightTicksPerRotation = ROBOT_DRIVE_DEFAULT_WHEEL_TICKS_PER_ROTATION;
  float strafeTicksPerRotation = ROBOT_DRIVE_DEFAULT_WHEEL_TICKS_PER_ROTATION;

  float distanceThreshold = ROBOT_DRIVE_DEFAULT_DISTANCE_THRESHOLD;
  float angleThreshold = ROBOT_DRIVE_DEFAULT_ANGLE_THRESHOLD;

  float heading = 0;

  bool waitingForTimer = false;

  bool inversed = false;

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
    pros::lcd::print(0, "X =  %f", currentX);
    pros::lcd::print(1, "Y =  %f", currentY);
    pros::lcd::print(2, "A =  %d", getAngleAsGyro());
    pros::lcd::print(3, "Left =  %f", getLeftDriveSensor());
    pros::lcd::print(4, "deltaA =  %f", gyroDirection(direction, getCurrentAngle()) * gyroDifference(direction, getCurrentAngle()));
    pros::lcd::print(5, "Strafe =  %f", getStrafeDriveSensor());
    pros::lcd::print(6, "Strafe =  %f", leftSpeed);
    pros::lcd::print(7, "Strafe =  %f", rightSpeed);

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
          systemTimer.zeroTimer();
          drivePIDTimer.stopTimer();
          drivePIDTimer.zeroTimer();

          driveTurnPID.resetPID();
          distancePID.resetPID();
          turnPID.resetPID();

          driveTurnPID.setPIDTarget(0);
          distancePID.setPIDTarget(0);
          turnPID.setPIDTarget(0);

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

            case driveThrough:
            driveCommand = systemCommands[systemReadPos];
            systemReadPos++;
            targetX = systemCommands[systemReadPos];
            systemReadPos++;
            targetY = systemCommands[systemReadPos];
            systemReadPos++;
            break;

            case turnToPoint:
            driveCommand = systemCommands[systemReadPos];
            systemReadPos++;
            targetX = systemCommands[systemReadPos];
            systemReadPos++;
            targetY = systemCommands[systemReadPos];
            systemReadPos++;
            break;

            case turnToAngle:
            driveCommand = systemCommands[systemReadPos];
            systemReadPos++;
            targetAngle = systemCommands[systemReadPos];
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
      switch (driveCommand)
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
        if (drivePIDTimer.currentTime() > 2000)
        {
          leftSpeed = 0;
          rightSpeed = 0;
          this->systemDone = true;
        }
        break;

        case driveThrough:

        deltaX = targetX - currentX;
        deltaY = targetY - currentY;
        direction = atan(deltaY / deltaX);
        distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

        leftSpeed = rightSpeed = 127;

        leftSpeed += (fabs(leftSpeed) * driveTurnPID.calculatePID(gyroDirection(direction, getCurrentAngle()) * gyroDifference(direction, getCurrentAngle())));
        rightSpeed -= (fabs(rightSpeed) * driveTurnPID.calculatePID(gyroDirection(direction, getCurrentAngle()) * gyroDifference(direction, getCurrentAngle())));

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

        leftSpeed = turnPID.calculatePID(gyroDirection(direction, getCurrentAngle()) * gyroDifference(direction, getCurrentAngle()));
        rightSpeed = -leftSpeed;

        if (direction - getCurrentAngle() <= angleThreshold)
        {
          drivePIDTimer.startTimer();
        }
        if (drivePIDTimer.currentTime() > 2000)
        this->systemDone = true;
        break;

        case turnToAngle:
        direction = targetAngle;

        leftSpeed = turnPID.calculatePID(gyroDirection(direction, getCurrentAngle()) * gyroDifference(direction, getCurrentAngle()));
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
      moveDrive(leftSpeed, rightSpeed);
    }
    else
    {
      systemMaxTime = 0;
      moveDrive(0, 0);
    }
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
    if(!inversed)
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
    else
    {
      if ((leftEncoder != nullptr && rightEncoder != nullptr) && !useGyroInsteadOfFreeEncoders)
      return correctGyroValue(getAngleAsGyro() - 1800);
      else if (driveGyro != nullptr)
      return correctGyroValue(this->driveGyro->get_value() - 1800);
      else if (leftMotors[0] != nullptr && rightMotors[0] != nullptr)
      return correctGyroValue(getAngleAsGyro() - 1800);
      else
      return 0;
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
  bool addStrafeMotor(driveMotor *newMotor)
  {
    for (int i = 0; i < TOTAL_MOTORS; i++)
    {
      if (centerMotors[i] == nullptr)
      {
        strafeMotors[i] = newMotor;
        return true;
      }
    }
    return false;
  }
  bool addCenterMotor(driveMotor *newMotor)
  {
    for (int i = 0; i < TOTAL_MOTORS; i++)
    {
      if (centerMotors[i] == nullptr)
      {
        centerMotors[i] = newMotor;
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
  float getStrafeDriveSensor()
  {
    if (strafeEncoder != nullptr)
    {
      return strafeEncoder->get_value() - strafeDriveSensorLastReset;
    }
    else
    {
      if (strafeMotors[0] != nullptr)
      return strafeMotors[0]->get_position() - strafeDriveSensorLastReset;
      else
      return 0;
    }
  }

  void moveDrive(int left, int right)
  {
    float tempLeftIn;
    float tempRightIn;
    if(inversed)
    {
      tempLeftIn = -right;
      tempRightIn = -left;
      left = tempLeftIn;
      right = tempRightIn;
    }
    switch (driveType)
    {
      case STRAIGHT_DRIVE:
      moveStraightDrive(left, right);
      break;
      case TRIANGLE_DRIVE:
      moveTriangleDrive(left, right);
      break;

      default:
      break;
    }
  }

  void moveStraightDrive(int left, int right)
  {
    for (int i = 0; i < TOTAL_MOTORS; i++)
    {
      if (leftMotors[i] != nullptr)
      {
        leftMotors[i]->setRequestedSpeed(left);
      }
    }
    for (int i = 0; i < TOTAL_MOTORS; i++)
    {
      if (rightMotors[i] != nullptr)
      {
        rightMotors[i]->setRequestedSpeed(right);
      }
    }
  }

  void moveTriangleDrive(int left, int right)
  {
    for (int i = 0; i < TOTAL_MOTORS; i++)
    {
      if (leftMotors[i] != nullptr)
      {
        leftMotors[i]->setRequestedSpeed(left);
      }
    }
    for (int i = 0; i < TOTAL_MOTORS; i++)
    {
      if (rightMotors[i] != nullptr)
      {
        rightMotors[i]->setRequestedSpeed(right);
      }
    }
    for (int i = 0; i < TOTAL_MOTORS; i++)
    {
      if (centerMotors[i] != nullptr)
      {
        centerMotors[i]->setRequestedSpeed((left + right) / 2);
      }
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

  uint32_t previousVelocityCheck = pros::c::millis();

  float getwheelDistanceMoved(float ticksMoved, float wheelDiameter, float ticksPerRotation)
  {
    return (ticksMoved / ticksPerRotation) * (wheelDiameter * PI);
  }

  int getAngleAsGyro()
  {
    return correctGyroValue(((currentAngle * (180 / PI)) * 10));
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
    angleVelocity = ((currentAngle - previousAngle) * 1000) / previousVelocityCheck;
    xVelocity = ((currentX - previousX) * 1000) / previousVelocityCheck;
    yVelocity = ((currentY - previousY) * 1000) / previousVelocityCheck;

    previousAngle = currentAngle;
    previousX = currentX;
    previousY = currentY;
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

    currentY += distanceTraveledStrafe * -sinAngleTraveled;
    currentX += distanceTraveledStrafe * cosAngleTraveled;

    currentAngle += angleChange;
  }
};

#endif
