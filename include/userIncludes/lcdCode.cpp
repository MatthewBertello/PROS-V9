#pragma once

#ifndef lcdCode_cpp
#define lcdCode_cpp

#include "headers/autonomous.h"
#include "pros/llemu.hpp"
#include "headers/lcdCode.h"
#include "headers/globals.h"

bool leftButtonPressed = false;
bool centerButtonPressed = false;
bool rightButtonPressed = false;

void autonomousSelection()
{
  clearLines();
  if (autonomousSelected)
  {
    pros::lcd::print(0, "Autonomous %d selected", autonomousMode);
    pros::lcd::set_text(1, "Center Btn to deselect");
    pros::lcd::register_btn1_cb(unselectAutonomous);
  }
  else
  {
    pros::lcd::print(0, "Autonomous %d", autonomousMode);
    pros::lcd::print(1, "Center Btn to select",0);
    pros::lcd::register_btn0_cb(decreaseAutonomousMode);
    pros::lcd::register_btn1_cb(selectAutonomous);
    pros::lcd::register_btn2_cb(increaseAutonomousMode);
  }
}

void decreaseAutonomousMode()
{
  leftButtonPressed = !leftButtonPressed;
  if (leftButtonPressed)
  {
    autonomousMode--;
    if (autonomousMode < 1)
      autonomousMode = MAX_AUTONOMOUS_MODES;
  }
}

void selectAutonomous()
{
  centerButtonPressed = !centerButtonPressed;
  if (centerButtonPressed)
  {
    autonomousSelected = true;
  }
}

void increaseAutonomousMode()
{
  rightButtonPressed = !rightButtonPressed;
  if (rightButtonPressed)
  {
    autonomousMode++;
    if (autonomousMode > MAX_AUTONOMOUS_MODES)
      autonomousMode = 1;
  }
}

void unselectAutonomous()
{
  centerButtonPressed = !centerButtonPressed;
  if (centerButtonPressed)
  {
    autonomousSelected = false;
  }
}

void clearLines()
{
  pros::lcd::clear_line(0);
  pros::lcd::clear_line(1);
  pros::lcd::clear_line(2);
  pros::lcd::clear_line(3);
  pros::lcd::clear_line(4);
  pros::lcd::clear_line(5);
  pros::lcd::clear_line(6);
  pros::lcd::clear_line(7);
}

#endif
