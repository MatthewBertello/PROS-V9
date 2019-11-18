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

  driveLayouts driveType = STRAIGHT_DRIVE;

  driveMotor *leftMotors[TOTAL_MOTORS] = {};
  driveMotor *rightMotors[TOTAL_MOTORS] = {};
  driveMotor *strafeMotors[TOTAL_MOTORS] = {};

  bool hasGyro = false;

  pros::ADIGyro *driveGyro = nullptr;
  pros::ADIEncoder *leftEncoder = nullptr;
  pros::ADIEncoder *rightEncoder = nullptr;
  pros::ADIEncoder *strafeEncoder = nullptr;


  PIDController leftDrivePID = PIDController(.1, .05, .01, 5, 254, 0, 635, true, true);
  PIDController rightDrivePID = PIDController(.1, .05, .01, 5, 254, 0, 635, true, true);
  PIDController drivePID = PIDController(.1, 0, 0, 10, 0, 0, 0, true, true);
  PIDController gyroPID = PIDController(.1, .025, .1, 10, 800, 0, 300, true, true);
  PIDController straightGyroPID = PIDController(.0025, .00, .0, 0, 1000, 0, 500, true, true);

  timer drivePIDTimer;

  float leftSpeed = 0;// left drive speed
  float rightSpeed = 0;// right drive speed
  float strafeSpeed = 0;
  int chassisDirection = 0;
  int strafeDirection = 0;
  float chassisSpeed = 0;
  float chassisDistance = 0;
  float strafeDistance = 0;
  int largestDriveInput = 127;

  int driveThreshold = 25;
  int gyroThreshold = 50;

  float heading = 0;

  void setDriveType(driveLayouts driveType)
  {
    this->driveType = driveType;
  }

  void addGyro(pros::ADIGyro *gyro)
  {
    driveGyro = gyro;
    hasGyro = true;
  }

  float getGyroSensor()
  {
    if(hasGyro)
    return this->driveGyro->get_value();
    else
    return 0;
  }

  bool addLeftMotor(driveMotor *newMotor)
  {
    for (int i = 0; i < 7; i++)
    {
      if(leftMotors[i] == nullptr)
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
      if(rightMotors[i] == nullptr)
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
      if(leftMotors[i] == nullptr)
      {
        strafeMotors[i] = newMotor;
        return true;
      }
    }
    return false;
  }

  void resetLeftDriveSensor()
  {
    if(leftMotors[0] != nullptr)
    leftMotors[0]->getMotorPointer()->tare_position();
  }

  void resetRightDriveSensor()
  {
    if(leftMotors[0] != nullptr)
    rightMotors[0]->getMotorPointer()->get_position();
  }

  float getLeftDriveSensor()
  {
    if(leftMotors[0] != nullptr)
    return leftMotors[0]->getMotorPointer()->get_position();
    else
    return 0;
  }

  float getRightDriveSensor()
  {
    if(rightMotors[0] != nullptr)
    return rightMotors[0]->getMotorPointer()->get_position();
    else
    return 0;
  }

  float getStrafeMotorSensor()
  {
    if(strafeMotors[0] != nullptr)
    return strafeMotors[0]->getMotorPointer()->get_position();
    else
    return 0;
  }

  void moveStraightDrive(int left, int right)
  {
    for (int i = 0; i < 7; i++)
    {
      if(leftMotors[i] != nullptr)
      {
        leftMotors[i]->setRequestedSpeed(left);
      }
    }
    for (int i = 0; i < 7; i++)
    {
      if(rightMotors[i] != nullptr)
      {
        rightMotors[i]->setRequestedSpeed(right);
      }
    }
  }

  void moveStrafeDrive(int left, int right, int strafeSpeed)
  {
    this->moveStraightDrive(left, right);
    for (int i = 0; i < 7; i++)
    {
      if(strafeMotors[i] != nullptr)
      {
        strafeMotors[i]->setRequestedSpeed(speed);
      }
    }
  }

  void executeSytemFunction()
  {
    if(!this->systemDone)// if the drive is not done
    {
      if(chassisDirection == driveStraight)// if the robot should drive straight
      {
        rightDrivePID.setPIDTarget(chassisDistance);
        leftDrivePID.setPIDTarget(chassisDistance);
        leftSpeed = leftDrivePID.calculatePID(getLeftDriveSensor());
        leftSpeed = rightDrivePID.calculatePID(getRightDriveSensor());

        if (fabs(getRightDriveSensor()) > fabs(getLeftDriveSensor()))// if the right drive has travelled farther than the left drive
        {
          drivePID.resetPID();
          drivePID.setPIDTarget(getLeftDriveSensor());
          rightSpeed = rightSpeed - fabs(drivePID.calculatePID(getRightDriveSensor()));
        }
        else if (fabs(getLeftDriveSensor()) > fabs(getRightDriveSensor()))// if the left drive has travelled farther than the left drive
        {
          drivePID.resetPID();
          drivePID.setPIDTarget(getRightDriveSensor());
          rightSpeed = rightSpeed - fabs(drivePID.calculatePID(getLeftDriveSensor()));
        }

        if((fabs(getRightDriveSensor() - chassisDistance) <= driveThreshold) || (fabs(getLeftDriveSensor() - chassisDistance) <= driveThreshold))
        drivePIDTimer.startTimer();

        if(drivePIDTimer.currentTime() > 300)
        this->systemDone = true;
      }

      else if(chassisDirection == gyroDriveStraight)// otherwise if the robot should drive straight with the gyro
      {
        heading = correctGyroValue(heading);
        straightGyroPID.setPIDTarget(0);
        rightDrivePID.setPIDTarget(chassisDistance);
        leftDrivePID.setPIDTarget(chassisDistance);
        leftSpeed = leftDrivePID.calculatePID(getLeftDriveSensor());
        rightSpeed = rightDrivePID.calculatePID(getRightDriveSensor());
        if(getLeftDriveSensor() > getRightDriveSensor())
        chassisSpeed = getRightDriveSensor();
        else
        chassisSpeed = getLeftDriveSensor();

        if(getLeftDriveSensor() > getRightDriveSensor())
        leftSpeed = rightSpeed;
        else
        rightSpeed = leftSpeed;

        if(fabs(leftSpeed) > largestDriveInput)
        leftSpeed = largestDriveInput*sgn(leftSpeed);
        if(fabs(rightSpeed) > largestDriveInput)
        rightSpeed = largestDriveInput*sgn(rightSpeed);

        if((gyroDirection(getGyroSensor(), heading) == 1))// if the robot is drifting left
        {
          leftSpeed += (fabs(leftSpeed) * fabs(straightGyroPID.calculatePID(gyroDifference(getGyroSensor(), heading))));
          rightSpeed -= (fabs(rightSpeed) * fabs(straightGyroPID.calculatePID(gyroDifference(getGyroSensor(), heading))));
          // leftSpeed +=fabs(leftSpeed)*.5;
          // rightSpeed -=fabs(rightSpeed)*.5;
          // leftSpeed += fabs(calculatePID(&straightGyroPID, gyroDifference(getGyroSensor(), heading)));
          // rightSpeed -= fabs(calculatePID(&straightGyroPID, gyroDifference(getGyroSensor(), heading)));
        }
        else if((gyroDirection(getGyroSensor(), heading) == -1))//otherwise if the robot is drifting right
        {
          leftSpeed -= (fabs(leftSpeed) * fabs(straightGyroPID.calculatePID(gyroDifference(getGyroSensor(), heading))));
          rightSpeed += (fabs(rightSpeed) * fabs(straightGyroPID.calculatePID(gyroDifference(getGyroSensor(), heading))));
          // leftSpeed -=fabs(leftSpeed)*.5;
          // rightSpeed +=fabs(rightSpeed)*.5;
          // leftSpeed -= fabs(calculatePID(&straightGyroPID, gyroDifference(getGyroSensor(), heading)));
          // rightSpeed += fabs(calculatePID(&straightGyroPID, gyroDifference(getGyroSensor(), heading)));
        }

        if((fabs(chassisSpeed - chassisDistance) <= driveThreshold))
        {
          drivePIDTimer.startTimer();
        }
        if(drivePIDTimer.currentTime() > 300)
        this->systemDone = true;
      }

      else if(chassisDirection == gyroTurn)// if the robot should turn using the gyro sensor
      {
        chassisDistance = correctGyroValue(chassisDistance);// correct the gyro target value
        gyroPID.setPIDTarget(0);
        if((gyroDirection(getGyroSensor(), chassisDistance) == 1))
        {
          leftSpeed = fabs(gyroPID.calculatePID(gyroDifference(getGyroSensor(), chassisDistance)*gyroDirection(getGyroSensor(), chassisDistance)));
          rightSpeed = -fabs(gyroPID.calculatePID(gyroDifference(getGyroSensor(), chassisDistance)*gyroDirection(getGyroSensor(), chassisDistance)));

        }
        else if((gyroDirection(getGyroSensor(), chassisDistance) == -1))
        {
          leftSpeed = -fabs(gyroPID.calculatePID(gyroDifference(getGyroSensor(), chassisDistance)*gyroDirection(getGyroSensor(), chassisDistance)));
          rightSpeed = fabs(gyroPID.calculatePID(gyroDifference(getGyroSensor(), chassisDistance)*gyroDirection(getGyroSensor(), chassisDistance)));
        }
        if(fabs(getGyroSensor() - chassisDistance) <= gyroThreshold)
        {
          drivePIDTimer.startTimer();
        }

        if(drivePIDTimer.currentTime() > 300)
        this->systemDone = true;

      }

      else if(chassisDirection == sweepRight || chassisDirection == forwardRight)
      {
        rightDrivePID.setPIDTarget(chassisDistance);
        chassisSpeed = rightDrivePID.calculatePID(getRightDriveSensor());;
        if(chassisDirection == sweepRight)
        {
          rightSpeed = chassisSpeed;
          leftSpeed = chassisSpeed*.5;
        }
        else
        {
          rightSpeed = chassisSpeed;
          leftSpeed = 0;
        }
        if((fabs(getRightDriveSensor() - chassisDistance) <= driveThreshold))
        {
          drivePIDTimer.startTimer();
        }
        if(drivePIDTimer.currentTime() > 300)
        this->systemDone = true;
      }
      else if(chassisDirection == sweepLeft || chassisDirection == forwardLeft)// otherwise if the robot is driving with the left drive
      {
        leftDrivePID.setPIDTarget(chassisDistance);
        chassisSpeed = leftDrivePID.calculatePID(getLeftDriveSensor());
        if(chassisDirection == sweepLeft)
        {
          rightSpeed = chassisSpeed*.5;
          leftSpeed = chassisSpeed;
        }
        else
        {
          rightSpeed = 0;
          leftSpeed = chassisSpeed;
        }
        if((fabs(getLeftDriveSensor() - chassisDistance) <= driveThreshold))
        {
          drivePIDTimer.startTimer();
        }
        if(drivePIDTimer.currentTime() > 300)
        this->systemDone = true;
      }

      else if(chassisDirection == driveSpeed)
      {
        leftSpeed = chassisSpeed;
        rightSpeed = chassisSpeed;
        if(fabs(getLeftDriveSensor()) >= fabs(chassisDistance) && fabs(getRightDriveSensor()) >= fabs(chassisDistance))
        this->systemDone = true;
      }

      else if(chassisDirection == turnSpeed)
      {
        leftSpeed = chassisSpeed;
        rightSpeed = -chassisSpeed;

        if(fabs(getLeftDriveSensor()) >= fabs(chassisDistance) && fabs(getRightDriveSensor()) >= fabs(chassisDistance))
        this->systemDone = true;
      }

      else if(chassisDirection == sweepLeftSpeed || chassisDirection == forwardLeftSpeed)
      {
        leftSpeed = chassisSpeed;

        if(chassisDirection == sweepLeft)
        rightSpeed = chassisSpeed*.5;

        else
        rightSpeed = 0;


        if(fabs(getLeftDriveSensor()) >= fabs(chassisDistance))
        this->systemDone = true;
      }

      else if(chassisDirection == sweepRightSpeed || chassisDirection == forwardRightSpeed)
      {
        rightSpeed = chassisSpeed;

        if(chassisDirection == sweepRight)
        leftSpeed = chassisSpeed*.5;

        else
        leftSpeed = 0;


        if(fabs(getRightDriveSensor()) >= fabs(chassisDistance))
        this->systemDone = true;
      }

      else if(chassisDirection == driveStraightSpeed)
      {
        leftSpeed = chassisSpeed;
        rightSpeed = chassisSpeed;

        if (fabs(getRightDriveSensor()) > fabs(getLeftDriveSensor()))// if the right drive has travelled farther than the left drive
        {
          drivePID.setPIDTarget(getLeftDriveSensor());
          rightSpeed += drivePID.calculatePID(getRightDriveSensor());
        }
        else if (fabs(getLeftDriveSensor()) > fabs(getRightDriveSensor()))// if the right drive has travelled farther than the left drive
        {
          drivePID.setPIDTarget(getRightDriveSensor());
          rightSpeed += drivePID.calculatePID(getLeftDriveSensor());
        }

        if(fabs(getLeftDriveSensor()) >= fabs(chassisDistance) && fabs(getRightDriveSensor()) >= fabs(chassisDistance))
        this->systemDone = true;
      }

      else if(chassisDirection == gyroDriveStraightSpeed)
      {
        heading = correctGyroValue(heading);
        straightGyroPID.setPIDTarget(0);
        leftSpeed = chassisSpeed;
        rightSpeed = chassisSpeed;

        if((gyroDirection(getGyroSensor(), heading) == 1))// if the robot is drifting left
        {
          leftSpeed += fabs(leftSpeed) * fabs(straightGyroPID.calculatePID(gyroDifference(getGyroSensor(), heading)*gyroDirection(getGyroSensor(), heading)));
          rightSpeed -= fabs(rightSpeed) * fabs(straightGyroPID.calculatePID(gyroDifference(getGyroSensor(), heading)*gyroDirection(getGyroSensor(), heading)));
        }
        else if((gyroDirection(getGyroSensor(), heading) == -1))//otherwise if the robot is drifting right
        {
          leftSpeed -= fabs(leftSpeed) * fabs(straightGyroPID.calculatePID(gyroDifference(getGyroSensor(), heading)*gyroDirection(getGyroSensor(), heading)));
          rightSpeed += fabs(rightSpeed) * fabs(straightGyroPID.calculatePID(gyroDifference(getGyroSensor(), heading)*gyroDirection(getGyroSensor(), heading)));
        }


        if(fabs(getLeftDriveSensor()) >= fabs(chassisDistance) && fabs(getRightDriveSensor()) >= fabs(chassisDistance))
        this->systemDone = true;

      }

      else if(chassisDirection == gyroTurnSpeed)
      {
        heading = correctGyroValue(heading);
        chassisDistance = correctGyroValue(chassisDistance);// correct the gyro target value

        if((gyroDirection(getGyroSensor(), chassisDistance) == 1))
        {
          leftSpeed = chassisSpeed;
          rightSpeed = -chassisSpeed;
        }
        else if((gyroDirection(getGyroSensor(), chassisDistance) == -1))
        {
          leftSpeed = -chassisSpeed;
          rightSpeed = chassisSpeed;
        }

        if(gyroDifference(getGyroSensor(), heading) >= gyroDifference(heading, chassisDistance))
        this->systemDone = true;
      }

      else// otherwise
      {
        this->systemDone = true;// the drive is done
      }
      if(fabs(leftSpeed) > largestDriveInput)
      leftSpeed = largestDriveInput*sgn(leftSpeed);
      if(fabs(rightSpeed) > largestDriveInput)
      rightSpeed = largestDriveInput*sgn(rightSpeed);

      moveStraightDrive(leftSpeed, rightSpeed);
    }
    else// if the drive is done
    {
      moveStraightDrive(0,0);
    }
  }
};

#endif
