#pragma once
#ifndef pidController_h
#define pidController_h

#include "headers/mathFunctions.h"
#include <cmath>
#include "pros/rtos.hpp"

class PIDController
{
	float xp;
	float xi;
	float xd;
	float xc;

	float currentC;
	bool changeConstantDirection;

	float lastError;
	float error;

	float integral;
	float derivative;

	float integralLimit;
	bool resetIntegralAtCross;
	float lowerIntegralBand;
	float upperIntegralBand;

	float previousTime;
	float deltaTime;

	float target;

public:

	PIDController(float xp = DEFAULT_PID_P, float xi = DEFAULT_PID_I, float xd = DEFAULT_PID_D, float xc = DEFAULT_PID_C, float integralLimit = DEFAULT_PID_INTEGRAL_LIMIT,
	float lowerIntegralBand = DEFAULT_PID_LOWER_INTEGRAL_BAND, float upperIntegralBand = DEFAULT_PID_UPPER_INTEGRAL_BAND,  bool resetIntegralAtCross = DEFAULT_PID_RESET_INTEGRAL_AT_CROSS, 
	bool changeConstantDirection = DEFAULT_PID_CHANGE_CONSTANT_DIRECTION)
	{
		this->integralLimit = integralLimit;
		this->lowerIntegralBand = lowerIntegralBand;

		this->upperIntegralBand = upperIntegralBand;
		this->resetIntegralAtCross = resetIntegralAtCross;

		this->changeConstantDirection = changeConstantDirection;

		this->previousTime = pros::c::millis();
		this->target = 0;

		this->xp = xp;
		this->xi = xi;
		this->xd = xd;
		this->xc = xc;
		this->currentC = xc;

		this->lastError = 0;

		this->integral = 0;
		this->derivative = 0;

	}

	float calculatePID(float currentValue)
	{
		this->deltaTime = (pros::c::millis()-this->previousTime)/1000;
		this->previousTime = pros::c::millis();

		this->error = this->target-currentValue;

		if(fabs(this->error) > this->lowerIntegralBand && fabs(this->error) < this->upperIntegralBand)
		this->integral += (this->error*this->deltaTime);

		if(fabs(this->integral) > this->integralLimit)
		this->integral = this->integralLimit*sgn(this->integral);

		if(this->resetIntegralAtCross)
		{
			if(this->error == 0 || sgn(this->error) != sgn(this->lastError))
			this->integral = 0;
		}

		this->derivative = (this->error-this->lastError)*this->deltaTime;
		this->lastError = this->error;

		if(this->changeConstantDirection)
		this->currentC = fabs(this->xc) * sgn(this->error);

		float output = this->error*this->xp + this->integral*this->xi - this->derivative*this->xd + this->currentC;
		return output;
	}

	void resetPID()
	{
		this->lastError = 0;
		this->integral = 0;
		this->derivative = 0;
		this->previousTime = 0;
		this->deltaTime = 0;
		this->target = 0;
	}

	void setPIDTarget(float target)
	{
		this->target = target;
	}
};

#endif
