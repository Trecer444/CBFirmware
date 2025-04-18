/*
 * settings.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Admin
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#include "stdint.h"
#include <string>

struct chSpec
{
	uint16_t
        signalSource,
        pwmValue,
        flashType,
        flashCount,
        heater1,
        heater2,
        delayTimerValue,
        shutdownTimerValue,
        vCutOffValue,        //сантивольты (делить на 100)
        vAutoEnValue,        //сантивольты (делить на 100)
        flashFreq,            //*100
        currCutOffValue;      //*100

    uint8_t
        engineOn,
        shutdownTimer,
        vCutOff,
        vAutoEn,
        pwm,
        currCutOff,
        delayTimer,
        flash;
};

typedef struct chSettingsEEPROM
{
	uint16_t
        delayTimerValue[6],
        shutdownTimerValue[6],
        vCutOffValue[6],
        vAutoEnValue[6],
        flashFreq[6],
        currCutOffValue[6],
		checkboxes[6];			//хранит биты булевые переменных в порядке от младшего к старшему engineOn shutdownTimer vCutOff vAutoEn pwm currCutOff delayTimer flash
	uint8_t
		signalSource[6],
        pwmValue[6],
        flashType[6],
        flashCount[6],
        heater1[6],
        heater2[6];

};

class Param
{
private:
    //параметры платы
	chSpec chSettings[6];
public:
    Param();
    void setDefaultVal();
    uint8_t setParam(char *paramString, uint16_t stringSize);
    uint8_t setParam(char *paramString);
    uint8_t getParam(char *paramString, uint16_t stringSize);
    uint8_t getParam(char *paramString);
    uint8_t getSourceSign(uint8_t index);

};




#endif /* INC_SETTINGS_H_ */
