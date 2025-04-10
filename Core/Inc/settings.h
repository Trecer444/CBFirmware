/*
 * settings.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Admin
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#include "stdint.h"

struct Param
{
    //параметры платы
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

uint8_t setParam (char* datastr, struct Param* par);


#endif /* INC_SETTINGS_H_ */
