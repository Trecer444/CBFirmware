/*
 * settings.cpp
 *
 *  Created on: Apr 10, 2025
 *      Author: Admin
 */

#include "settings.h"


void Param::setDefaultVal()
{
	for (uint8_t i = 0; i < 6; i++)
	{
		this->chSettings[i].signalSource = 0;
		this->chSettings[i].pwmValue = 50;
		this->chSettings[i].flashType = 0;
		this->chSettings[i].flashCount = 5;
		this->chSettings[i].heater1 = 20;
		this->chSettings[i].heater2 = 50;
		this->chSettings[i].delayTimerValue = 0;
		this->chSettings[i].shutdownTimerValue = 300;
		this->chSettings[i].vCutOffValue = 1250;			//сантивольты (делить на 100)
		this->chSettings[i].vAutoEnValue = 1280;        	//сантивольты (делить на 100)
		this->chSettings[i].flashFreq = 200;            	//*100
		this->chSettings[i].currCutOffValue = 100;      	//*100

		this->chSettings[i].engineOn = 0;
		this->chSettings[i].shutdownTimer = 0;
		this->chSettings[i].vCutOff = 0;
		this->chSettings[i].vAutoEn = 0;
		this->chSettings[i].pwm = 0;
		this->chSettings[i].currCutOff = 0;
		this->chSettings[i].delayTimer = 0;
		this->chSettings[i].flash = 0;
	}
}

Param::Param()
{
	this->setDefaultVal();
}

uint8_t Param::setParam(char *paramString)
{
	uint16_t charLenght = 0;
	while (paramString[charLenght] != '*')
	{
		charLenght++;
	}
	uint16_t arr[20];
	for (uint8_t k = 0; k < 20; k++) arr[k] = 0;
	uint8_t ch = 0, index = 0;
	for (uint16_t i = 0; i < charLenght - 1; i++)
	{
		if (paramString[i]=='*' || paramString[i] == '\n')			//если нашли конец строки то завершаем
			break;
		if (paramString[i] == '$' && i < charLenght - 2)
		{
			i++;
			ch = paramString[i] - '0';
			index = 0;
			i++;
			continue;
		}
		if (paramString[i] == ' ')
		{
			index++;
			continue;
		}
		if (paramString[i] >= '0' && paramString[i] <= '9')
		{
			arr[index] = arr[index]*10 + (paramString[i] - '0');
			continue;
		}
		if (paramString[i] == ';')
		{
			this->chSettings[ch].signalSource = arr[0];
			this->chSettings[ch].pwmValue = arr[1];
			this->chSettings[ch].flashType = arr[2];
			this->chSettings[ch].flashCount = arr[3];
			this->chSettings[ch].heater1 = arr[4];
			this->chSettings[ch].heater2 = arr[5];
			this->chSettings[ch].delayTimerValue = arr[6];
			this->chSettings[ch].shutdownTimerValue = arr[7];
			this->chSettings[ch].vCutOffValue = arr[8];			//сантивольты (делить на 100)
			this->chSettings[ch].vAutoEnValue = arr[9];        	//сантивольты (делить на 100)
			this->chSettings[ch].flashFreq = arr[10];            	//*100
			this->chSettings[ch].currCutOffValue = arr[11];      	//*100

			this->chSettings[ch].engineOn = arr[12];
			this->chSettings[ch].shutdownTimer = arr[13];
			this->chSettings[ch].vCutOff = arr[14];
			this->chSettings[ch].vAutoEn = arr[15];
			this->chSettings[ch].pwm = arr[16];
			this->chSettings[ch].currCutOff = arr[17];
			this->chSettings[ch].delayTimer = arr[18];
			this->chSettings[ch].flash = arr[19];
			for (uint8_t k = 0; k < 20; k++) arr[k] = 0;
		}

	}
	return 0;
}

uint8_t Param::getSourceSign(uint8_t index)
{
	return this->chSettings[index].signalSource;

}
