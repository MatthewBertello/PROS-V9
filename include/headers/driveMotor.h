#pragma once
#ifndef driveMotor_h
#define driveMotor_h

#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "headers/config.h"


class driveMotor{

  int requestedSpeed;
  int currentSpeed;
  int slewRate;
  bool shouldSlew;
  pros::Motor *motorPointer;


public:

  driveMotor()
  {
    this->requestedSpeed = 0;
    this->currentSpeed = 0;
    this->slewRate = DEFAULT_SLEW_RATE;
    this->shouldSlew = true;
    this->motorPointer = nullptr;
  }

  int getRequestedSpeed()
  {
    return this->requestedSpeed;
  }

  int getCurrentSpeed()
  {
    return this->currentSpeed;
  }

  int getSlewRate()
  {
    return this->slewRate;
  }

  bool slewEnabled()
  {
    return this->shouldSlew;
  }

  pros::Motor *getMotorPointer()
  {
    return this->motorPointer;
  }

  void setRequestedSpeed(int requestedSpeed)
  {
    this->requestedSpeed = requestedSpeed;
  }

  void setCurrentSpeed(int currentSpeed)
  {
    this->currentSpeed = currentSpeed;
  }

  void setSlewRate(int slewRate)
  {
    this->slewRate = slewRate;
  }

  void disableSlew()
  {
    this->shouldSlew = false;
  }

  void enableSlew()
  {
    this->shouldSlew = true;
  }

  void setMotorPointer(pros::Motor motor)
  {
    this->motorPointer = &motor;
  }

  void setMotor(int speed)
  {
    this->requestedSpeed = speed;
  }

  void bypassSlew(int speed)
  {
    this->currentSpeed = speed;
    this->requestedSpeed = speed;
  }

  // void slewMotors(void* param)
  // {
  //   driveMotors[0].setMotorPointer(motorOne);
  //   driveMotors[1].setMotorPointer(motorTwo);
  //   driveMotors[2].setMotorPointer(motorThree);
  //   driveMotors[3].setMotorPointer(motorFour);
  //   driveMotors[4].setMotorPointer(motorFive);
  //   driveMotors[5].setMotorPointer(motorSix);
  //   driveMotors[6].setMotorPointer(motorSeven);
  //   driveMotors[7].setMotorPointer(motorEight);
  //   driveMotors[7].getMotorPointer();
  //
  //   uint32_t lastRun = pros::c::millis();
  //   int motorIndex = 0;
  //   driveMotor *x;
  //
  //   while(true)
  //   {
  //     while(motorIndex < TOTAL_MOTORS)
  //     {
  //       x = &(driveMotors[motorIndex]);
  //
  //       if(x->slewEnabled())
  //       {
  //         if(x->getCurrentSpeed() < x->getRequestedSpeed())
  //         {
  //           x->setCurrentSpeed(x->getCurrentSpeed() + x->getSlewRate());
  //           x->setCurrentSpeed((x->getCurrentSpeed() > x->getRequestedSpeed()) ? x->getRequestedSpeed():x->getCurrentSpeed());
  //         }
  //         if(x->getCurrentSpeed() > x->getRequestedSpeed())
  //         {
  //           x->setCurrentSpeed(x->getCurrentSpeed() - x->getSlewRate());
  //           x->setCurrentSpeed((x->getCurrentSpeed() < x->getRequestedSpeed()) ? x->getRequestedSpeed():x->getCurrentSpeed());
  //         }
  //         x->setCurrentSpeed((x->getCurrentSpeed() > MAX_MOTOR_SPEED) ? MAX_MOTOR_SPEED:x->getCurrentSpeed());
  //         x->setCurrentSpeed((x->getCurrentSpeed() < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED:x->getCurrentSpeed());
  //
  //         x->getMotorPointer()->move(x->getCurrentSpeed());
  //       }
  //       else
  //       {
  //         x->getMotorPointer()->move(x->getRequestedSpeed());
  //       }
  //       motorIndex++;
  //     }
  //     motorIndex = 0;
  //     pros::c::task_delay_until(&lastRun, SLEW_REFRESH_RATE);
  //     lastRun = pros::c::millis();
  //   }
  // }

};

#endif
