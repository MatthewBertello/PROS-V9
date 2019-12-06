#pragma once
#ifndef systems_h
#define systems_h

#include "headers/timer.h"
#include <vector>

class systems
{

public:

  bool systemDone = true;
  bool systemCompleted;
  timer systemTimer = timer();
  int systemMaxTime = 0;
  int systemReadPos = 0;

  std::vector<int> systemCommands;

  virtual int addSystemCommands(int i, std::vector<int> &commands)
  {
  }
  virtual void executeSystemFunction()
  {
  }
  virtual bool updateSystem()
  {
  }
  void resetSystemCommads()
  {
    systemCommands.clear();
    systemReadPos = 0;
  }
};

#endif
