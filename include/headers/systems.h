#pragma once
#ifndef systems_h
#define systems_h

#include "headers/timer.h"

class systems
{

public:

  enum systemTypes
  {
    DRIVE
  };
  systemTypes systemType;
  bool systemDone;
  timer systemTimer = timer();
  int systemMaxTime;


virtual void executeSystemFunction()
{
}

};

#endif
