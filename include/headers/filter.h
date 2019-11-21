#pragma once
#ifndef filters_h
#define filters_h

#include "headers/config.h"
#include <cmath>

class emaFilter
{
	int period = DEFAULT_EMA_FILTER_PERIOD;
	float alpha;;
	float currentValue = 0;
	float previousValue = 0;

public:

	emaFilter()
	{
		this->alpha = (2/((float)this->period+1));
	}

	emaFilter(int length, float initialValue)
	{
		this->period = length;
		this->currentValue = initialValue;
		this->previousValue = initialValue;
		this->alpha = (2/((float)length+1));
	}

	float filterEma(float input)
	{
		this->currentValue = ((input-this->previousValue) * this->alpha) + this->previousValue;
		this->previousValue = this->currentValue;
		return this->currentValue;
	}

	float getEma(emaFilter *filter)
	{
		return this->currentValue;
	}
};

class demaFilter
{
	int period1 = DEFAULT_DEMA_FILTER_PERIOD;
	float alpha1;
	float currentValue1 = 0;
	float previousValue1 = 0;
	int period2 = DEFAULT_DEMA_FILTER_PERIOD;
	float alpha2;
	float currentValue2 = 0;
	float previousValue2 = 0;
	float output;

public:

	demaFilter()
	{
		this->alpha1 = (2/((float)this->period1+1));
		this->alpha2 = (2/((float)this->period2+1));
	}

	demaFilter(int length1, int length2, float initialValue)
	{
		this->period1 = length1;
		this->currentValue1 = initialValue;
		this->alpha1 = (2/((float)length1+1));
		this->period2 = length2;
		this->currentValue2 = initialValue;
		this->alpha2 = (2/((float)length2+1));
		this->output = initialValue;
	}

	float filterDema(demaFilter *filter, float input)
	{
		filter->currentValue1 = ((input-filter->previousValue1) * filter->alpha1) + filter->previousValue1;
		filter->previousValue1 = filter->currentValue1;

		filter->currentValue2 = ((filter->currentValue1-filter->previousValue2) * filter->alpha2) + filter->previousValue2;
		filter->previousValue2 = filter->currentValue2;

		filter->output = 2*filter->currentValue1-filter->currentValue2;
		return filter->output;
	}

	float getDema(demaFilter *filter)
	{
		return filter->output;
	}

};

class medianFilter
{
	int width = MEDIAN_FILTER_DEFAULT_WIDTH;
	float previousValues[MEDIAN_FILTER_MAX_VALUES] = {0};
	float currentValue;
public:

	medianFilter(int width, float initialValue)
	{
		width = width;
		currentValue = initialValue;
		for (int i = 0; i < MEDIAN_FILTER_MAX_VALUES; i++)
		{
			previousValues[i] = initialValue;
		}
	}

	float filterMedian(float input)
	{
		float tempArray[MEDIAN_FILTER_MAX_VALUES];
		int y = this->width-1;
		while(y > 0)
		{
			tempArray[y] = this->previousValues[y-1];
			y--;
		}
		tempArray[0] = (int)input;
		for(int count = 0; count < this->width; ++count)
		{
			this->previousValues[count] = tempArray[count];
		}

		y = 1;
		int j = 0;
		int testing;
		int largest = tempArray[0];
		while(y < this->width)
		{
			testing = tempArray[y];
			j = y-1;

			while(j >= 0 && tempArray[j] > testing)
			{
				tempArray[j+1] = tempArray[j];
				j -=1;
			}
			tempArray[j+1] = testing;
			y++;
		}
		this->currentValue = tempArray[static_cast<int>(floor(this->width/2))];
		return this->currentValue;

	}

	float getMedian(medianFilter *filter)
	{
		medianFilter *x = (filter);
		return this->currentValue;
	}

};

#endif
