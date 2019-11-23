#include "main.h"
#include "headers/opcontrol.h"
#include "headers/initialize.h"
#include "headers/lcdCode.h"
#include "headers/robotDrive.h"

/**
* Runs the operator control code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the operator
* control mode.
*
* If no competition control is connected, this function will run immediately
* following initialize().
*
* If the robot is disabled or communications is lost, the
* operator control task will be stopped. Re-enabling the robot will restart the
* task, not resume it from where it left off.
*/
bool curve = false;    // should the robot use the parametric drive formula
bool inversed = false; // should the drive be inversed
bool control = false;  // should the drive be in control mode

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Controller partner(pros::E_CONTROLLER_PARTNER);

void opcontrol()
{
  uint32_t lastRun = pros::c::millis();
  // pros::lcd::print(0, "X =  %d", mainDrive.currentX);
  // pros::lcd::print(1, "Y =  %d", mainDrive.currentY);
  // pros::lcd::print(2, "A =  %d", mainDrive.currentAngle);
  // pros::lcd::print(3, "Left =  %d", mainDrive.getLeftDriveSensor());
  // pros::lcd::print(4, "Right =  %d", mainDrive.getRightDriveSensor());
  // pros::lcd::print(5, "Strafe =  %d", mainDrive.getStrafeDriveSensor());

  while (true) // infinite while loop
  {
    pros::lcd::print(0, "X =  %d", mainDrive.currentX);
    pros::lcd::print(1, "Y =  %d", mainDrive.currentY);
    pros::lcd::print(2, "A =  %d", mainDrive.currentAngle);
    pros::lcd::print(3, "Left =  %d", leftEncoder.get_value());
    pros::lcd::print(4, "Right =  %d", mainDrive.getAbsoluteRightDriveSensor());
    pros::lcd::print(5, "Strafe =  %d", strafeEncoder.get_value());
    // autonomousSelection();

    if (partner.is_connected()) // if their are two controllers
    {
      doubleControllerDrive(); // use the two controller drive code
    }
    else // if there is one controller
    {
      singleControllerDrive(); // use the one controller drive code
    }
    pros::c::task_delay_until(&lastRun, OP_CONTROL_REFRESH_RATE);
    lastRun = pros::c::millis();
  }
}

void doubleControllerDrive()
{
  float leftIn;
  float rightIn;
  float tempLeftIn;
  float tempRightIn;

  leftIn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  rightIn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

  if (control)
  {
    leftIn = (leftIn * .5);
    rightIn = (rightIn * .5);
  }

  if (curve)
  {
    leftIn = fabs(leftIn) * leftIn / 127;
    rightIn = fabs(rightIn) * rightIn / 127;
  }

  if (inversed)
  {
    tempLeftIn = -rightIn;
    tempRightIn = -leftIn;
    leftIn = tempLeftIn;
    rightIn = tempRightIn;
  }

  mainDrive.moveStraightDrive(leftIn, rightIn);
}

void singleControllerDrive()
{
  doubleControllerDrive();
}
