#include "main.h"
#include "headers/opcontrol.h"
#include "headers/initialize.h"
#include "headers/lcdCode.h"
#include "headers/robotDrive.h"
#include "headers/taskFunctions.h"

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
  // pros::Task mainDrivePositionTrackerTask(mainDrivePositionTrackerFn, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "mainDrivePositionTrackerTask");

  uint32_t lastRun = pros::c::millis();
  // pros::lcd::print(0, "X =  %d", mainDrive.currentX);
  // pros::lcd::print(1, "Y =  %d", mainDrive.currentY);
  // pros::lcd::print(2, "A =  %d", mainDrive.currentAngle);
  // pros::lcd::print(3, "Left =  %d", mainDrive.getLeftDriveSensor());
  // pros::lcd::print(4, "Right =  %d", mainDrive.getLeftDriveSensor());
  // pros::lcd::print(5, "Strafe =  %d", mainDrive.getLeftDriveSensor());

  while (true) // infinite while loop
  {
    // if(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) > 0)
    // {
    mainDrive.leftF = rightEncoder2.get_value();
    // }
    mainDrive.trackPosition();
    int thx = mainDrive.currentX;
    int thy = mainDrive.currentY;
    int tha = mainDrive.getAngleAsGyro();
    int thrwd = mainDrive.rightWheelDistance;
    int thrw = mainDrive.angleChange;
    pros::lcd::print(0, "X =  %d", thx);
    pros::lcd::print(1, "Y =  %d",thy);
    pros::lcd::print(2, "A =  %d", tha);
    pros::lcd::print(3, "left =  %d", mainDrive.leftF);
    pros::lcd::print(4, "Rightd =  %d", rightEncoder2.get_value());
    // pros::lcd::print(5, "angc =  %d", thrw);
    // pros::lcd::print(5, "Strafe =  %d", mainDrive.getLeftDriveSensor());
    // autonomousSelection();
    

    if (partner.is_connected()) // if their are two controllers
    {
      doubleControllerDrive(); // use the two controller drive code
    }
    else // if there is one controller
    {
      singleControllerDrive(); // use the one controller drive code
    }
    pros::c::task_delay_until(&lastRun, 500);
    lastRun = pros::c::millis();
  }
}

void doubleControllerDrive()
{
  float leftIn;
  float rightIn;
  float tempLeftIn;
  float tempRightIn;

  leftIn = 0;
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
