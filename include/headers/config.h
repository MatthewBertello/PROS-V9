#pragma once
#ifndef config_h
#define config_h

//system identifiers for systems in systemsArray
enum activeSystems
{
    base
};

//motor ports
extern const int FIRSTPORT;
extern const int SECONDPORT;
extern const int THIRDPORT;
extern const int FOURTHPORT;
extern const int FIFTHPORT;
extern const int SEVENTHPORT;
extern const int SIXTHPORT;
extern const int EIGTHPORT;

extern const int FRONTLEFTDRIVE;  // port 1
extern const int FRONTRIGHTDRIVE; // port 2
extern const int BACKLEFTDRIVE;   // port 3
extern const int BACKRIGHTDRIVE;  // port 4
extern const int RAMP;     // port 5
extern const int LEFTINTAKE;     // port 6
extern const int RIGHTINTAKE;         // port 7
extern const int LIFTMOTOR;       // port 8

//driveMotor.h
extern const float DEFAULT_SLEW_RATE;
extern const bool SHOULD_SLEW_BY_DEFAULT;

//filter.h
extern const int DEFAULT_EMA_FILTER_PERIOD;
extern const int DEFAULT_DEMA_FILTER_PERIOD;
extern const int MEDIAN_FILTER_DEFAULT_WIDTH;
#define MEDIAN_FILTER_MAX_VALUES 100

//initialize.h
#define NUMBER_OF_SYSTEMS 1
#define TOTAL_MOTORS 8

//pidController.h
extern const float DEFAULT_PID_P;
extern const float DEFAULT_PID_I;
extern const float DEFAULT_PID_D;
extern const float DEFAULT_PID_C;
extern const float DEFAULT_PID_INTEGRAL_LIMIT;
extern const float DEFAULT_PID_LOWER_INTEGRAL_BAND;
extern const float DEFAULT_PID_UPPER_INTEGRAL_BAND;
extern const bool DEFAULT_PID_RESET_INTEGRAL_AT_CROSS;
extern const bool DEFAULT_PID_CHANGE_CONSTANT_DIRECTION;

//robotDrive.h
extern const float ROBOT_DRIVE_DEFAULT_WHEEL_DIAMETER;
extern const float ROBOT_DRIVE_DEFAULT_WHEEL_DISTANCE_FROM_CENTER;
extern const float ROBOT_DRIVE_DEFAULT_WHEEL_TICKS_PER_ROTATION;
extern const float ROBOT_DRIVE_DEFAULT_DISTANCE_THRESHOLD;
extern const float ROBOT_DRIVE_DEFAULT_ANGLE_THRESHOLD;

//lcdCode.cpp
extern const int MAX_AUTONOMOUS_MODES;

//autonomous.cpp
extern const bool DEFAULT_AUTONOMOUS_MODE;

//taskFunctions.cpp
extern const int SLEW_REFRESH_RATE;
extern const int MAX_MOTOR_SPEED;
extern const int MIN_MOTOR_SPEED;
extern const int DRIVE_TRACKER_REFRESH_RATE;

//opControl.cpp
extern const int OP_CONTROL_REFRESH_RATE;

#endif
