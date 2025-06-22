/*
 * status.cpp
 *
 *  Created on: Jun 21, 2025
 *      Author: Admin
 */

#include <status.h>

status::status()
{
	engineRunStatus = 0;
	voltage = 0;

	bIgnition = 0;
	bLoBeam = 0;
	bHiBeam = 0;
	bAnyBeam = 0;
	bRightTurner = 0;
	bLeftTurner = 0;
	bAnyTurner = 0;
	bEmergencyLight = 0;
	bHeater1 = 0;
	bHeater2 = 0;
	bStopLight = 0;
}

status::~status() {
	// TODO Auto-generated destructor stub
}



uint16_t status::getVoltage() { return voltage; }
uint8_t status::getIgnition() { return bIgnition; }

uint8_t status::getLoBeam() { return bLoBeam; }
uint8_t status::getHiBeam() { return bHiBeam; }
uint8_t status::getAnyBeam() { return bAnyBeam; }

uint8_t status::getRightTurner() { return bRightTurner; }
uint8_t status::getLeftTurner() { return bLeftTurner; }
uint8_t status::getAnyTurner() { return bAnyTurner; }

uint8_t status::getEmergencyLight() { return bEmergencyLight; }

uint8_t status::getHeater1() { return bHeater1; }
uint8_t status::getHeater2() { return bHeater2; }

uint8_t status::getStopLight() { return bStopLight; }

// Реализация set-методов

void status::setVoltage(uint16_t v) 	{ voltage = v; }

void status::setIgnitionOn() 			{ bIgnition = 1; }
void status::setIgnitionOff() 			{ bIgnition = 0; }

void status::setLoBeamOn() 				{ bLoBeam = 1; updateAnyBeam(); }
void status::setLoBeamOff() 			{ bLoBeam = 0; updateAnyBeam(); }

void status::setHiBeamOn() 				{ bHiBeam = 1; updateAnyBeam(); }
void status::setHiBeamOff() 			{ bHiBeam = 0; updateAnyBeam(); }

void status::setRightTurnerOn() 		{ bRightTurner = 1; updateAnyTurner(); }
void status::setRightTurnerOff() 		{ bRightTurner = 0; updateAnyTurner(); }

void status::setLeftTurnerOn() 			{ bLeftTurner = 1; updateAnyTurner(); }
void status::setLeftTurnerOff() 		{ bLeftTurner = 0; updateAnyTurner(); }

void status::setEmergencyLightOn() 		{ bEmergencyLight = 1; }
void status::setEmergencyLightOff() 	{ bEmergencyLight = 0; }

void status::setbHeater1On() 			{ bHeater1 = 1; }
void status::setHeater1Off() 			{ bHeater1 = 0; }

void status::setHeater2On() 			{ bHeater2 = 1; }
void status::setHeater2Off() 			{ bHeater2 = 0; }

void status::setStopLightOn() 			{ bStopLight = 1; }
void status::setStopLightOff() 			{ bStopLight = 0; }

//вспомогательные методы (для света и поворотников)
void status::updateAnyBeam() { bAnyBeam = (bLoBeam || bHiBeam) ? 1 : 0; }
void status::updateAnyTurner() { bAnyTurner = (bLeftTurner || bRightTurner) ? 1 : 0; }
