#pragma once
#ifndef config_h
#define config_h

#define NUMBER_OF_SYSTEMS 10
#define TOTAL_MOTORS 8

extern const int FIRSTPORT;
extern const int SECONDPORT;
extern const int THIRDPORT;
extern const int FOURTHPORT;
extern const int FIFTHPORT;
extern const int SEVENTHPORT;
extern const int SIXTHPORT;
extern const int EIGTHPORT;

extern const int FRONTLEFTDRIVE; // port 1
extern const int FRONTRIGHTDRIVE; // port 2
extern const int BACKLEFTDRIVE; // port 3
extern const int BACKRIGHTDRIVE; // port 4
extern const int STRAFEWHEEL; // port 5
extern const int INTAKEMOTOR; // port 6
extern const int SHOOTER; // port 7
extern const int LIFTMOTOR; // port 8

extern const int DEFAULT_SLEW_RATE;
extern const int SLEW_REFRESH_RATE;
extern const int MAX_MOTOR_SPEED;
extern const int MIN_MOTOR_SPEED;

#endif
