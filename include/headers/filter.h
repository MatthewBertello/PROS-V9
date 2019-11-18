#pragma once
#ifndef filters_h
#define filters_h

#include <cmath>

class emaFilter
{
	int period;
	float alpha;
	float currentValue;
	float previousValue;

public:

	emaFilter()
	{
		this->period = 10;
		this->currentValue = 0;
		this->previousValue = 0;
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
	int period1;
	float alpha1;
	float currentValue1;
	float previousValue1;
	int period2;
	float alpha2;
	float currentValue2;
	float previousValue2;
	float output;

public:

	demaFilter()
	{
		this->period1 = 10;
		this->currentValue1 = 0;
		this->alpha1 = (2/((float)this->period1+1));
		this->period2 = 10;
		this->currentValue2 = 0;
		this->alpha2 = (2/((float)this->period2+1));
		this->output = 0;
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
	int width;
	float previousValues[100];
	float currentValue;
public:

	medianFilter()
	{
		this->width = 10;
		this->currentValue = 0;
		int y = 0;
		while(y < 50)
		{
			this->previousValues[y] = 0;
			y++;
		}
	}

	medianFilter(int width, float initialValue)
	{
		this->width = width;
		this->currentValue = initialValue;
		int y = 0;
		while(y < 50)
		{
			this->previousValues[y] = initialValue;
			y++;
		}
	}

	float filterMedian(float input)
	{
		float tempArray[50];
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

	void resetMedianFilter(float initialValue)
	{
		this->currentValue = initialValue;
		int y = 0;
		while(y < 50)
		{
			this->previousValues[y] = initialValue;
			y++;
		}
	}

	float getMedian(medianFilter *filter)
	{
		medianFilter *x = (filter);
		return this->currentValue;
	}

};

#endif
