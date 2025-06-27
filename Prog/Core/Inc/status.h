/*
 * status.h
 * Глобальный статус состояний мотоцикла.
 * По сути в нем хранятся состояния всех входов, при этом этот класс мы передаем в outputch, и там он обрабатывается уже каждым каналом отдельно.
 *
 *  Created on: Jun 21, 2025
 *      Author: Admin
 */

#ifndef INC_STATUS_H_
#define INC_STATUS_H_

#define STATUS_MSG_TIMEOUT		100 //через сколько циклов по 10 мс (таймер 3) статус сменится на 0, если не было вызова метода включения

#include "stdint.h"

class status {
private:
	uint16_t
		voltage,
		currents[6];
	uint8_t
		bIgnition,
		bLoBeam,
		bHiBeam,
		bAnyBeam,
		bRightTurner,
		bLeftTurner,
		bAnyTurner,
		bEmergencyLight,
		bHeater,			//единственная небулева переменная, может принимать 0(выкл), 1 и 2 (ступени подогрева)
		bStopLight,
		engineStatus;		//включен ли двигатель

	uint16_t
		timerEngineOn,
		timerIgnition,
		timerLoBeam,
		timerHiBeam,
		timerLeftTurner,
		timerRightTurner,
		timerEmergencyLight,
		timerHeater,
		timerStoplight;

	void updateAnyBeam();
	void updateAnyTurner();

public:
	status();
	virtual ~status();
	void setCurrents(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, uint16_t ch5);
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
	void setHeaterOn(uint8_t val); //1 или 2 ступень
	void setHeaterOff();
	void setStopLightOn();
	void setStopLightOff();
	void setEngineOn();
	void setEngineOff();


	uint16_t getCurrent(uint8_t chInd);
	uint16_t getVoltage();
	uint8_t getIgnitionStatus();
	uint8_t getLoBeam();
	uint8_t getHiBeam();
	uint8_t getAnyBeam(); // если есть флаг bAnyBeam
	uint8_t getRightTurner();
	uint8_t getLeftTurner();
	uint8_t getAnyTurner(); // если есть флаг bAnyTurner
	uint8_t getEmergencyLight();
	uint8_t getHeater();
	uint8_t getStopLight();
	uint8_t getEngineStatus();

	void updateStatusTimers();
};
#endif /* INC_STATUS_H_ */

