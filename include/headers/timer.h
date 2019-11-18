#pragma once
#ifndef timer_h
#define timer_h

#include "pros/rtos.hpp"

class timer
{
	int initialValue;
	int timerValue;
	bool paused;

public:

	timer()
	{
		this->timerValue = 0;
		this->paused = false;
		this->initialValue = pros::c::millis();
	}

	int currentTime()
	{
		if(!(this->paused))
		return (pros::c::millis() - this->initialValue);
		else
		return (this->timerValue);
	}

	void zeroTimer()
	{
		this->timerValue = 0;
		this->initialValue = pros::c::millis();
	}

	void stopTimer()
	{
		if(!this->paused)
		this->timerValue = (pros::c::millis() - this->initialValue);

		this->paused = true;
	}

	void startTimer()
	{
		if(this->paused)
		this->initialValue = pros::c::millis() - this->timerValue;

		this->paused = false;
	}
};

#endif
