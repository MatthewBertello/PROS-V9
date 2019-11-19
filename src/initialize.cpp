#include "main.h"

#include "headers/config.h"
#include "headers/driveMotor.h"
#include "headers/robotDrive.h"
#include "headers/robotFunction.h"
#include "headers/autonomous.h"
#include "headers/opcontrol.h"

pros::Motor motorOne(FIRSTPORT);
pros::Motor motorTwo(SECONDPORT);
pros::Motor motorThree(THIRDPORT);
pros::Motor motorFour(FOURTHPORT);
pros::Motor motorFive(FIFTHPORT);
pros::Motor motorSix(SIXTHPORT);
pros::Motor motorSeven(SEVENTHPORT);
pros::Motor motorEight(EIGTHPORT);

driveMotor driveMotors[TOTAL_MOTORS];

robotDrive mainDrive;

pros::ADIGyro driveGyro(1);

robotFunction autonRobotFunction;
systems *systemsArray[NUMBER_OF_SYSTEMS];

#include "userIncludes/taskFunctions.cpp"

pros::Task autonomousInUserControlTask(autonomousInUserControlFn, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "autonomousInUserControlTask");
pros::Task slewMotorsTask(slewMotors, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "slewMotorsTask");
pros::Task mainDrivePositionTrackerTask(mainDrivePositionTrackerFn, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "mainDrivePositionTrackerTask");

#include "userIncludes/autons.cpp"
#include "userIncludes/config.cpp"
#include "userIncludes/gyroFunctions.cpp"
#include "userIncludes/lcdCode.cpp"
#include "userIncludes/mathFunctions.cpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{
  pros::lcd::initialize();

  driveMotors[0].setMotorPointer(motorOne);
  driveMotors[1].setMotorPointer(motorTwo);
  driveMotors[2].setMotorPointer(motorThree);
  driveMotors[3].setMotorPointer(motorFour);
  driveMotors[4].setMotorPointer(motorFive);
  driveMotors[5].setMotorPointer(motorSix);
  driveMotors[6].setMotorPointer(motorSeven);
  driveMotors[7].setMotorPointer(motorEight);

  driveMotors[0].getMotorPointer()->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[1].getMotorPointer()->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[2].getMotorPointer()->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[3].getMotorPointer()->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[4].getMotorPointer()->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[5].getMotorPointer()->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[6].getMotorPointer()->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  driveMotors[7].getMotorPointer()->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  driveMotors[0].disableSlew();
  driveMotors[1].disableSlew();
  driveMotors[2].disableSlew();
  driveMotors[3].disableSlew();
  driveMotors[4].disableSlew();
  driveMotors[5].disableSlew();
  driveMotors[6].disableSlew();
  driveMotors[7].disableSlew();

  driveMotors[FRONTLEFTDRIVE].getMotorPointer()->set_gearing(pros::E_MOTOR_GEARSET_18);
  driveMotors[FRONTRIGHTDRIVE].getMotorPointer()->set_gearing(pros::E_MOTOR_GEARSET_18);
  driveMotors[BACKLEFTDRIVE].getMotorPointer()->set_gearing(pros::E_MOTOR_GEARSET_18);
  driveMotors[BACKRIGHTDRIVE].getMotorPointer()->set_gearing(pros::E_MOTOR_GEARSET_18);
  driveMotors[INTAKEMOTOR].getMotorPointer()->set_gearing(pros::E_MOTOR_GEARSET_06);
  driveMotors[SHOOTER].getMotorPointer()->set_gearing(pros::E_MOTOR_GEARSET_36);
  driveMotors[LIFTMOTOR].getMotorPointer()->set_gearing(pros::E_MOTOR_GEARSET_36);

  driveMotors[FRONTRIGHTDRIVE].getMotorPointer()->set_reversed(true);
  driveMotors[BACKRIGHTDRIVE].getMotorPointer()->set_reversed(true);
  driveMotors[SHOOTER].getMotorPointer()->set_reversed(true);
  driveMotors[INTAKEMOTOR].getMotorPointer()->set_reversed(true);

  mainDrive.addLeftMotor(&driveMotors[FRONTLEFTDRIVE]);
  mainDrive.addLeftMotor(&driveMotors[BACKLEFTDRIVE]);
  mainDrive.addRightMotor(&driveMotors[FRONTRIGHTDRIVE]);
  mainDrive.addRightMotor(&driveMotors[BACKRIGHTDRIVE]);

  mainDrive.addGyro(&driveGyro);
  systemsArray[0] = &mainDrive;
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
