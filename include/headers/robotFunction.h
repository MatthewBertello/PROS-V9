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
	enum Commands
	{
		unWritten,
		goToStart,
		isASystem,
		isAnAction,
	};
	enum Actions
	{
		pause,
		pauseAll,
		maxTime,
		end
	};

	int instructions[300];
	int commandReadPos = 0;
	int commandWritePos = 0;
	bool finished = true;
	int currentSystem = 0;

	timer pauseTimer;
	float maxPauseTime = 0;
	bool allSystemsFinished = true;

	uint32_t lastRun = pros::c::millis();

	systems *allSystems[NUMBER_OF_SYSTEMS] = {};
	int thing[1];


	bool finishedReading()
	{
		if((commandReadPos == commandWritePos || commandReadPos == commandWritePos-1) && finished)
		return true;
		else
		return false;
	}
	void addSystemArray(systems *allSystems[])
	{
		for (int i = 0; i < NUMBER_OF_SYSTEMS; i++)
		{
			this->allSystems[i] = allSystems[i];
		}
	}

	void resetRobotFunction()
	{
		commandReadPos = 0;
		commandWritePos = 0;
		finished = true;
		int x = 0;
		while (x < 300)
		{
			instructions[x] = unWritten;
			x++;
		}
		for (int i = 0; i < NUMBER_OF_SYSTEMS; i++)
		{
			if (allSystems[i] != nullptr)
			{
				allSystems[i]->systemDone = true;
			}
		}
	}
	void addCommand(int a = -1, int b = -1, int c = -1, int dd = -1, int e = -1, int f = -1, int g = -1, int h = -1, int ii = -1, int j = -1, int k = -1, int l = -1)
	{
		bool added = false;
		bool shouldAdd = true;
		while (!added)
		{
			shouldAdd = true;
			int x = 1;
			while (x <= 12)
			{
				if ((commandWritePos + x) == commandReadPos)
				{
					shouldAdd = false;
				}
				x++;
			}
			if (commandWritePos + x >= 299)
			{
				if (shouldAdd)
				{
					instructions[commandWritePos] = goToStart;
					if (commandReadPos != 0)
						commandWritePos = 0;

					shouldAdd = false;
				}
			}
			if (shouldAdd)
			{
				instructions[commandWritePos] = a;
				instructions[commandWritePos + 1] = b;
				instructions[commandWritePos + 2] = c;
				instructions[commandWritePos + 3] = dd;
				instructions[commandWritePos + 4] = e;
				instructions[commandWritePos + 5] = f;
				instructions[commandWritePos + 6] = g;
				instructions[commandWritePos + 7] = h;
				instructions[commandWritePos + 8] = ii;
				instructions[commandWritePos + 9] = j;
				instructions[commandWritePos + 10] = k;
				instructions[commandWritePos + 11] = l;
				commandWritePos += 12;
				added = true;
			}
		}
	}

	void addSystemCommand(int a = -1, int b = -1, int c = -1, int dd = -1, int e = -1, int f = -1, int g = -1, int h = -1, int ii = -1, int j = -1, int k = -1)
	{
		bool added = false;
		bool shouldAdd = true;
		while (!added)
		{
			shouldAdd = true;
			int x = 1;
			while (x <= 12)
			{
				if ((commandWritePos + x) == commandReadPos)
				{
					shouldAdd = false;
				}
				x++;
			}
			if (commandWritePos + x >= 299)
			{
				if (shouldAdd)
				{
					instructions[commandWritePos] = goToStart;
					if (commandReadPos != 0)
						commandWritePos = 0;

					shouldAdd = false;
				}
			}
			if (shouldAdd)
			{
				instructions[commandWritePos] = isASystem;
				instructions[commandWritePos + 1] = a;
				instructions[commandWritePos + 2] = b;
				instructions[commandWritePos + 3] = c;
				instructions[commandWritePos + 4] = dd;
				instructions[commandWritePos + 5] = e;
				instructions[commandWritePos + 6] = f;
				instructions[commandWritePos + 7] = g;
				instructions[commandWritePos + 8] = h;
				instructions[commandWritePos + 9] = ii;
				instructions[commandWritePos + 10] = j;
				instructions[commandWritePos + 11] = k;
				commandWritePos += 12;
				added = true;
			}
		}
	}

	void addActionCommand(int a = -1, int b = -1, int c = -1, int dd = -1, int e = -1, int f = -1, int g = -1, int h = -1, int ii = -1, int j = -1, int k = -1)
	{
		bool added = false;
		bool shouldAdd = true;
		while (!added)
		{
			shouldAdd = true;
			int x = 1;
			while (x <= 12)
			{
				if ((commandWritePos + x) == commandReadPos)
				{
					shouldAdd = false;
				}
				x++;
			}
			if (commandWritePos + x >= 299)
			{
				if (shouldAdd)
				{
					instructions[commandWritePos] = goToStart;
					if (commandReadPos != 0)
						commandWritePos = 0;

					shouldAdd = false;
				}
			}
			if (shouldAdd)
			{
				instructions[commandWritePos] = isAnAction;
				instructions[commandWritePos + 1] = a;
				instructions[commandWritePos + 2] = b;
				instructions[commandWritePos + 3] = c;
				instructions[commandWritePos + 4] = dd;
				instructions[commandWritePos + 5] = e;
				instructions[commandWritePos + 6] = f;
				instructions[commandWritePos + 7] = g;
				instructions[commandWritePos + 8] = h;
				instructions[commandWritePos + 9] = ii;
				instructions[commandWritePos + 10] = j;
				instructions[commandWritePos + 11] = k;
				commandWritePos += 12;
				added = true;
			}
		}
	}

	void exectueRobotFunction()
	{
		while (true)
		{

			//Parse the instructions
			if ((commandReadPos != commandWritePos) && (commandReadPos + 1 != commandWritePos))
			{
				if (instructions[commandReadPos] == goToStart)
				{
					commandReadPos = 0;
				}
				if (instructions[commandReadPos] == unWritten)
				{
					commandReadPos++;
				}

				switch (instructions[commandReadPos])
				{
				case isASystem:
					currentSystem = instructions[commandReadPos + 1];
					switch (allSystems[currentSystem]->systemType)
					{
					case systems::DRIVE:
						if (allSystems[currentSystem]->systemDone)
						{
							commandReadPos++;
							commandReadPos++;
							robotDrive *a = (robotDrive *)allSystems[currentSystem];
							a->rightDrivePID.resetPID();
							a->leftDrivePID.resetPID();
							a->drivePID.resetPID();
							a->gyroPID.resetPID();
							a->straightGyroPID.resetPID();
							a->resetLeftDriveSensor();
							a->resetRightDriveSensor();
							a->systemTimer.zeroTimer();
							a->drivePIDTimer.stopTimer();
							a->drivePIDTimer.zeroTimer();

							a->systemMaxTime = 0;
							a->systemDone = false;
							a->heading = a->getGyroSensor();

							if (instructions[commandReadPos] <= robotDrive::PIDDriveCommandMaxValue)
							{
								a->chassisDirection = instructions[commandReadPos];
								commandReadPos++;
								if (a->chassisDirection == robotDrive::gyroDriveStraight)
								{
									a->heading = instructions[commandReadPos];
									commandReadPos++;
								}
								a->chassisDistance = instructions[commandReadPos];
								commandReadPos++;
							}
							else
							{
								a->chassisDirection = instructions[commandReadPos];
								commandReadPos++;
								a->chassisSpeed = instructions[commandReadPos];
								commandReadPos++;
								a->chassisDistance = instructions[commandReadPos];
								commandReadPos++;
								if (a->chassisDirection == robotDrive::gyroDriveStraightSpeed)
								{
									a->heading = instructions[commandReadPos];
									commandReadPos++;
								}
							}
						}
						break;
					}
					break; //system case end

				case isAnAction:
					switch (instructions[commandReadPos] + 1)
					{
					case pause:
						currentSystem = instructions[commandReadPos + 2];
						if (allSystems[currentSystem]->systemDone)
							commandReadPos += 3;
						break;

					case pauseAll:
						if (finished)
							commandReadPos += 2;
						break;

					case maxTime:
						commandReadPos += 2;
						currentSystem = instructions[commandReadPos];
						commandReadPos++;
						allSystems[currentSystem]->systemMaxTime = instructions[commandReadPos];
						commandReadPos++;
						break;

					case end:
						commandReadPos += 2;
						currentSystem = instructions[commandReadPos];
						allSystems[currentSystem]->systemDone = true;
						commandReadPos++;
						break;
					}
					break; //action case end
				}		   //system or action switch
			}
		}

		for (int i = 0; i < NUMBER_OF_SYSTEMS; i++)
		{
			if (allSystems[i] != nullptr)
			{
				allSystems[i]->executeSystemFunction();
			}
		}

		for (int i = 0; i < NUMBER_OF_SYSTEMS; i++)
		{
			if (allSystems[i] != nullptr)
			{
				if ((allSystems[i]->systemTimer.currentTime() > allSystems[i]->systemMaxTime) && (allSystems[i]->systemMaxTime != 0))
					allSystems[i]->systemDone = true;
			}
		}

		for (int i = 0; i < NUMBER_OF_SYSTEMS; i++)
		{
			allSystemsFinished = true;
			if (allSystems[i] != nullptr)
			{
				if (!allSystems[i]->systemDone)
					allSystemsFinished = false;
			}
		}
		finished = allSystemsFinished;

		pros::c::task_delay_until(&lastRun, 10);
		lastRun = pros::c::millis();
	}
};

#endif
