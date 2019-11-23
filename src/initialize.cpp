#include "main.h"

#include "headers/config.h"
#include "headers/driveMotor.h"
#include "headers/robotDrive.h"
#include "headers/robotFunction.h"
#include "headers/autonomous.h"
#include "headers/opcontrol.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

driveMotor motorOne(FIRSTPORT);
driveMotor motorTwo(SECONDPORT);
driveMotor motorThree(THIRDPORT);
driveMotor motorFour(FOURTHPORT);
driveMotor motorFive(FIFTHPORT);
driveMotor motorSix(SIXTHPORT);
driveMotor motorSeven(SEVENTHPORT);
driveMotor motorEight(EIGTHPORT);

driveMotor driveMotors[TOTAL_MOTORS] = {motorOne, motorTwo, motorThree, motorFour, motorFive, motorSix, motorSeven, motorEight};

robotDrive mainDrive;

pros::ADIGyro driveGyro(1);

pros::ADIEncoder leftEncoder(7, 8, true);
pros::ADIEncoder rightEncoder(5, 6, false);
pros::ADIEncoder strafeEncoder(3, 4, true);

robotFunction autonRobotFunction;
systems *systemsArray[NUMBER_OF_SYSTEMS] = {};

#include "userIncludes/taskFunctions.cpp"

pros::Task slewMotorsTask(slewMotors, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "slewMotorsTask");
pros::Task mainDrivePositionTrackerTask(mainDrivePositionTrackerFn, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "mainDrivePositionTrackerTask");

#include "userIncludes/autons.cpp"
#include "userIncludes/config.cpp"
#include "userIncludes/gyroFunctions.cpp"
#include "userIncludes/lcdCode.cpp"
#include "userIncludes/mathFunctions.cpp"

void initialize()
{
  pros::lcd::initialize();

  driveMotors[0].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[1].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[2].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[3].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[4].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[5].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[6].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[7].set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  driveMotors[FRONTLEFTDRIVE].set_gearing(pros::E_MOTOR_GEARSET_18);
  driveMotors[FRONTRIGHTDRIVE].set_gearing(pros::E_MOTOR_GEARSET_18);
  driveMotors[BACKLEFTDRIVE].set_gearing(pros::E_MOTOR_GEARSET_18);
  driveMotors[BACKRIGHTDRIVE].set_gearing(pros::E_MOTOR_GEARSET_18);
  driveMotors[INTAKEMOTOR].set_gearing(pros::E_MOTOR_GEARSET_06);
  driveMotors[SHOOTER].set_gearing(pros::E_MOTOR_GEARSET_36);
  driveMotors[LIFTMOTOR].set_gearing(pros::E_MOTOR_GEARSET_36);

  driveMotors[FRONTRIGHTDRIVE].set_reversed(true);
  driveMotors[BACKRIGHTDRIVE].set_reversed(true);
  driveMotors[SHOOTER].set_reversed(true);
  driveMotors[INTAKEMOTOR].set_reversed(true);

  mainDrive.addLeftMotor(&driveMotors[FRONTLEFTDRIVE]);
  mainDrive.addLeftMotor(&driveMotors[BACKLEFTDRIVE]);
  mainDrive.addRightMotor(&driveMotors[FRONTRIGHTDRIVE]);
  mainDrive.addRightMotor(&driveMotors[BACKRIGHTDRIVE]);

  mainDrive.addGyro(&driveGyro);
  systemsArray[base] = &mainDrive;

  mainDrive.addLeftEncoder(&leftEncoder);
  mainDrive.addRightEncoder(&rightEncoder);
  mainDrive.addStrafeEncoder(&strafeEncoder);

  autonRobotFunction = robotFunction();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
  pros::lcd::set_text(1, "Hello PROS User!d");
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
  while (true)
  {
    autonomousSelection();
    pros::delay(10);
  }
}
