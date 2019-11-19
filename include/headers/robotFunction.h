#pragma once

#ifndef robotFunction_h
#define robotFunction_h

#include "headers/config.h"
#include "headers/timer.h"
#include "headers/robotDrive.h"
#include "headers/systems.h"
#include "pros/rtos.hpp"

class robotFunction
{

public:

	bool finished = true;
	uint32_t lastRun = pros::c::millis();

	std::vector<int> robotFunctionCommands;

	systems *allSystems[NUMBER_OF_SYSTEMS] = {};

	void addSystemArray(systems *allSystems[])
	{
		for (int i = 0; i < NUMBER_OF_SYSTEMS; i++)
		{
			this->allSystems[i] = allSystems[i];
		}
	}

	void resetRobotFunction()
	{
		finished = true;
		robotFunctionCommands.clear();
		for (int i = 0; i < NUMBER_OF_SYSTEMS; i++)
		{
			if (allSystems[i] != nullptr)
			{
				allSystems[i]->resetSystemCommads();
			}
		}
	}

	template <typename... Args>
	void addCommands(Args... args)
	{
		robotFunctionCommands = {args...};
		for (int i = 0; i < robotFunctionCommands.size(); i++)
		{
			i = allSystems[robotFunctionCommands[i]]->addSystemCommands(i + 1, robotFunctionCommands);
		}
		executeRobotFunction();
	}

	void executeRobotFunction()
	{
		do
		{
			finished = true;
			for (int i = 0; i < NUMBER_OF_SYSTEMS; i++)
			{
				if (allSystems[i] != nullptr)
				{
					if (!allSystems[i]->updateSystem())
						finished = false;
				}
			}
			pros::c::task_delay_until(&lastRun, 5);
			lastRun = pros::c::millis();
		} while (!finished);
	}
};

#endif
