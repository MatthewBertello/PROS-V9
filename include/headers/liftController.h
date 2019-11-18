#pragma once

#ifndef liftController_h
#define liftController_h

#include "pros/rtos.hpp"
#include <cmath>
class liftController
{
  float liftHolding;
  float liftEven;
  float liftP;
  bool wasEven;
  bool learnHolding;
  int liftPrevious;
  int liftTarget;
  int liftMaxSpeed;
  int liftErrorThreshold;
  int liftUpperBound;
  int liftLowerBound;
  float liftChange;
  int liftRefresh;
  int liftControllerLastRun;
  bool monitorSpeed;

  liftController(float liftP, int liftEven = 0, int liftChange =1 , int liftErrorThreshold = 200, int liftUpperBound = 4095, int liftLowerBound = 0, int liftRefresh = 50, int liftMaxSpeed =100, bool learnHolding = true, bool monitorSpeed = false)
  {

    this->liftHolding = 0;
    this->liftP = liftP;
    this->liftEven = liftEven;
    this->wasEven = false;
    this->learnHolding = learnHolding;
    this->liftMaxSpeed = liftMaxSpeed;
    this->liftChange = liftChange;
    this->liftRefresh = liftRefresh;
    this->liftErrorThreshold = liftErrorThreshold;
    this->liftUpperBound = liftUpperBound;
    this->liftLowerBound = liftLowerBound;
    this->monitorSpeed = monitorSpeed;

  }

  void setLiftTarget(int target)
  {
    this->liftTarget = target;
    this->liftControllerLastRun = 0;
  }

  int getPower(int currentLiftPosition)//determines and sets variable holding power for the lift
  {
    if((pros::c::millis() - this->liftControllerLastRun) > this->liftRefresh)// if the minimum time between cycles has passed
    {
      this->liftControllerLastRun = pros::c::millis();
      // if the lift should be going down and is going up
      if((currentLiftPosition > (this->liftTarget+this->liftErrorThreshold)) && (currentLiftPosition > this->liftLowerBound) && (currentLiftPosition >= this->liftPrevious) && this->liftHolding > -127)
      {
        this->liftHolding = this->liftHolding - this->liftChange;// decrease the holding strength
        if(this->wasEven)// if the lift was in the threshold last cycle
        {
          this->wasEven = false;// wasEven = false
          if(this->liftEven > -127 && this->learnHolding)// if the neutral holding power can be decreased
          {
            this->liftEven = this->liftEven - this->liftChange;// decrease the neutral holding power
          }
        }
      }
      // otherwise if the lift is going down and should be going down
      else if((currentLiftPosition > (this->liftTarget+this->liftErrorThreshold))&&(currentLiftPosition > (this->liftLowerBound+this->liftErrorThreshold)) && (currentLiftPosition < this->liftPrevious) && this->liftHolding > -127)
      {
        if(monitorSpeed && abs(currentLiftPosition - this->liftPrevious)/pros::c::millis()-this->liftControllerLastRun > this->liftMaxSpeed)// if the lift is going too fast
        {
          this->liftHolding = this->liftHolding + this->liftChange;// increase the holding strength
        }
        else if(monitorSpeed && abs(currentLiftPosition - this->liftPrevious)/pros::c::millis()-this->liftControllerLastRun < this->liftMaxSpeed/2)// if the lift is not going fast enough
        {
          this->liftHolding = this->liftHolding - this->liftChange;// decrease the holding strength
        }
        if(this->wasEven)// if the lift was in the threshold last cycle
        {
          this->wasEven = false;// wasEven = false
          if(this->liftEven > -127 && this->learnHolding)// if the neutral holding power can be decreased
          {
            this->liftEven = this->liftEven - this->liftChange;// decrease the neutral holding power
          }
        }
      }
      // otherwise if the lift is going down and should be going up
      else if((currentLiftPosition < (this->liftTarget-this->liftErrorThreshold))&&(currentLiftPosition < (this->liftUpperBound-this->liftErrorThreshold)) && (currentLiftPosition <= this->liftPrevious) && this->liftHolding < 127)
      {
        this->liftHolding = this->liftHolding + this->liftChange;// increase the holding strength
        if(this->wasEven)// if the lift was in the threshold last cycle
        {
          this->wasEven = false;// wasEven = false
          if(this->liftEven < 127 && this->learnHolding)// if the neutral holding power can be increased
          {
            this->liftEven = this->liftEven + this->liftChange;// increase the neutral holding power
          }
        }
      }
      // otherwise if the lift is going up and should be going up
      else if((currentLiftPosition < (this->liftTarget-this->liftErrorThreshold))&&(currentLiftPosition < (this->liftUpperBound-this->liftErrorThreshold)) && (currentLiftPosition > this->liftPrevious) && this->liftHolding < 127)
      {
        if(monitorSpeed && abs(currentLiftPosition - this->liftPrevious)/pros::c::millis()-this->liftControllerLastRun > this->liftMaxSpeed)// if the lift is going too fast
        {
          this->liftHolding = this->liftHolding - this->liftChange;// decrease the holding strength
        }
        else if(monitorSpeed && abs(currentLiftPosition - this->liftPrevious)/pros::c::millis()-this->liftControllerLastRun < this->liftMaxSpeed/2)// if the lift is not going fast enough
        {
          this->liftHolding = this->liftHolding + this->liftChange;// increase the holding strength
        }
        if(this->wasEven)// if the lift was in the threshold last cycle
        {
          this->wasEven = false;// wasEven = false
          if(this->liftEven < 127 && this->learnHolding)// if the neutral holding power can be increased
          {
            this->liftEven = this->liftEven + this->liftChange;// increase the neutral holding power
          }
        }
      }
      // otherwise if the lift is in the threshold
      else if((currentLiftPosition < (this->liftTarget+this->liftErrorThreshold) && (currentLiftPosition > (this->liftTarget-this->liftErrorThreshold))))
      {
        this->liftHolding = this->liftEven;// set the holding power to neutral
        this->wasEven = true;// wasEven = true;
      }
      this->liftPrevious = currentLiftPosition;// record the current lift value to be used the next time the function is run
    }
    return this->liftHolding + (this->liftTarget - currentLiftPosition)*this->liftP;
  }
};

#endif
