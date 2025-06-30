/*
 * status.cpp
 *
 *  Created on: Jun 21, 2025
 *      Author: Admin
 */

#include <status.h>

status::status()
    : voltage(0),
      currents{0},
      bIgnition(0),
      bLoBeam(0),
      bHiBeam(0),
      bAnyBeam(0),
      bRightTurner(0),
      bLeftTurner(0),
      bAnyTurner(0),
      bEmergencyLight(0),
      bHeater(0),
      bStopLight(0),
      engineStatus(0),
      timerIgnition(0),
      timerLoBeam(0),
      timerHiBeam(0),
      timerLeftTurner(0),
      timerRightTurner(0),
      timerEmergencyLight(0),
      timerHeater(0),
      timerStoplight(0)
{}

status::~status() {
	// TODO Auto-generated destructor stub
}



uint16_t status::getVoltage() 				{ return voltage; }
uint16_t status::getCurrent(uint8_t chInd)
{
	if (chInd <= 5)
	{
		return currents[chInd];
	}
	return 65535;					//отключит канал
}

uint8_t status::getIgnitionStatus() 		{ return bIgnition; }
uint8_t status::getEngineStatus()		{return engineStatus;}

uint8_t status::getLoBeam() 		{ return bLoBeam; }
uint8_t status::getHiBeam() 		{ return bHiBeam; }
uint8_t status::getAnyBeam() 		{ return bAnyBeam; }

uint8_t status::getRightTurner() 	{ return bRightTurner; }
uint8_t status::getLeftTurner() 	{ return bLeftTurner; }
uint8_t status::getAnyTurner() 		{ return bAnyTurner; }

uint8_t status::getEmergencyLight() { return bEmergencyLight; }

uint8_t status::getHeater() 		{ return bHeater; }

uint8_t status::getStopLight() 		{ return bStopLight; }


// Реализация set-методов

void status::setVoltage(uint16_t v) 	{ voltage = v; }

void status::setCurrents(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, uint16_t ch5)
{
	currents[0] = ch0;
	currents[1] = ch1;
	currents[2] = ch2;
	currents[3] = ch3;
	currents[4] = ch4;
	currents[5] = ch5;
}

void status::updateStatusTimers()
{
	timerEngineOn++;
	timerIgnition++;
	timerLoBeam++;
	timerHiBeam++;
	timerLeftTurner++;
	timerRightTurner++;
	timerEmergencyLight++;
	timerHeater++;
	timerStoplight++;
	if (timerEngineOn > STATUS_MSG_TIMEOUT)
	{
		setEngineOff();
		timerEngineOn = 0;
	}

	if (timerIgnition > STATUS_MSG_TIMEOUT)
	{
		setIgnitionOff();
		timerIgnition = 0;
	}

	if (timerLoBeam > STATUS_MSG_TIMEOUT)
	{
		setLoBeamOff();
		timerLoBeam = 0;
	}

	if (timerHiBeam > STATUS_MSG_TIMEOUT)
	{
		setHiBeamOff();
		timerHiBeam = 0;
	}

	if (timerLeftTurner > STATUS_MSG_TIMEOUT)
	{
		setLeftTurnerOff();
		timerLeftTurner = 0;
	}

	if (timerRightTurner > STATUS_MSG_TIMEOUT)
	{
		setRightTurnerOff();
		timerRightTurner = 0;
	}

	if (timerEmergencyLight > STATUS_MSG_TIMEOUT)
	{
		setEmergencyLightOff();
		timerEmergencyLight = 0;
	}

	if (timerHeater > STATUS_MSG_TIMEOUT)
	{
		setHeaterOff();
		timerHeater = 0;
	}

	if (timerStoplight > STATUS_MSG_TIMEOUT)
	{
		setStopLightOff();
		timerStoplight = 0;
	}
}

void status::setIgnitionOn() 			{ bIgnition = 1; timerIgnition = 0;}
void status::setIgnitionOff() 			{ bIgnition = 0; }

void status::setLoBeamOn() 				{ bLoBeam = 1; updateAnyBeam(); timerLoBeam = 0;}
void status::setLoBeamOff() 			{ bLoBeam = 0; updateAnyBeam(); }

void status::setHiBeamOn() 				{ bHiBeam = 1; updateAnyBeam(); timerHiBeam = 0;}
void status::setHiBeamOff() 			{ bHiBeam = 0; updateAnyBeam(); }

void status::setRightTurnerOn() 		{ bRightTurner = 1; updateAnyTurner(); timerRightTurner = 0;}
void status::setRightTurnerOff() 		{ bRightTurner = 0; updateAnyTurner(); }

void status::setLeftTurnerOn() 			{ bLeftTurner = 1; updateAnyTurner(); timerLeftTurner = 0;}
void status::setLeftTurnerOff() 		{ bLeftTurner = 0; updateAnyTurner(); }

void status::setEmergencyLightOn() 		{ bEmergencyLight = 1; timerEmergencyLight = 0;}
void status::setEmergencyLightOff() 	{ bEmergencyLight = 0; }

void status::setHeaterOn(uint8_t val)	{ bHeater = val; timerHeater = 0;}
void status::setHeaterOff() 			{ bHeater = 0; }

void status::setStopLightOn() 			{ bStopLight = 1; timerStoplight = 0;}
void status::setStopLightOff() 			{ bStopLight = 0; }

void status::setEngineOn() 				{ engineStatus = 1; timerEngineOn = 0;}
void status::setEngineOff() 			{ engineStatus = 0; }

//вспомогательные методы (для света и поворотников)
void status::updateAnyBeam() { bAnyBeam = (bLoBeam || bHiBeam) ? 1 : 0; }
void status::updateAnyTurner() { bAnyTurner = (bLeftTurner || bRightTurner) ? 1 : 0; }
