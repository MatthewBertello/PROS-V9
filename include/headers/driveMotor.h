#pragma once
#ifndef driveMotor_h
#define driveMotor_h

#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "headers/config.h"

class driveMotor : public pros::Motor
{

  int requestedSpeed = 0;
  int currentSpeed = 0;
  float slewRate = DEFAULT_SLEW_RATE;
  bool shouldSlew = SHOULD_SLEW_BY_DEFAULT;

public:
  driveMotor(int port) : pros::Motor(port)
  {
  }

  int getRequestedSpeed()
  {
    return this->requestedSpeed;
  }

  int getCurrentSpeed()
  {
    return this->currentSpeed;
  }

  float getSlewRate()
  {
    return this->slewRate;
  }

  bool slewEnabled()
  {
    return this->shouldSlew;
  }

  void setRequestedSpeed(int requestedSpeed)
  {
    this->requestedSpeed = requestedSpeed;
  }

  void setCurrentSpeed(int currentSpeed)
  {
    this->currentSpeed = currentSpeed;
  }

  void disableSlew()
  {
    this->shouldSlew = false;
  }

  void enableSlew()
  {
    this->shouldSlew = true;
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
};

#endif
