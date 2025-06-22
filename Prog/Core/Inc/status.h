/*
 * status.h
 *
 *  Created on: Jun 21, 2025
 *      Author: Admin
 */

#ifndef INC_STATUS_H_
#define INC_STATUS_H_

#include "stdint.h"

class status {
private:
	uint8_t
		engineRunStatus;
	uint16_t
		voltage;
	uint8_t
		bIgnition,
		bLoBeam,
		bHiBeam,
		bAnyBeam,
		bRightTurner,
		bLeftTurner,
		bAnyTurner,
		bEmergencyLight,
		bHeater1,
		bHeater2,
		bStopLight;
	void updateAnyBeam();
	void updateAnyTurner();

public:
	status();
	virtual ~status();
	void setVoltage(uint16_t v);
	void setIgnitionOn();
	void setIgnitionOff();
	void setLoBeamOn();
	void setLoBeamOff();
	void setHiBeamOn();
	void setHiBeamOff();
	void setRightTurnerOn();
	void setRightTurnerOff();
	void setLeftTurnerOn();
	void setLeftTurnerOff();
	void setEmergencyLightOn();
	void setEmergencyLightOff();
	void setbHeater1On();
	void setHeater1Off();
	void setHeater2On();
	void setHeater2Off();
	void setStopLightOn();
	void setStopLightOff();


	uint16_t getVoltage();
	uint8_t getIgnition();
	uint8_t getLoBeam();
	uint8_t getHiBeam();
	uint8_t getAnyBeam(); // если есть флаг bAnyBeam
	uint8_t getRightTurner();
	uint8_t getLeftTurner();
	uint8_t getAnyTurner(); // если есть флаг bAnyTurner
	uint8_t getEmergencyLight();
	uint8_t getHeater1();
	uint8_t getHeater2();
	uint8_t getStopLight();
};
#endif /* INC_STATUS_H_ */

