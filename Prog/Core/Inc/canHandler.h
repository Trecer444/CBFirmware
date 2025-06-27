/*
 * canHandler.h
 *
 *  Created on: Jun 23, 2025
 *      Author: Admin
 */

#ifndef INC_CANHANDLER_H_
#define INC_CANHANDLER_H_

#include "outputch.h"
#include "settings.h"
#include "status.h"
#include "stm32f4xx_hal_can.h"

//список обрабатываемых сообщений

// ID CAN сообщений
#define CAN_ID_ENGINE_ON 			0x10C 	//проверено
#define ENGINE_ON_BYTE				3
#define ENGINE_VAL_OFF				0x0		//по сути тут передаются обороты, так что все что не ноль истино

#define CAN_ID_IGNITION				0x130 	//просто наличие пакета

#define CAN_ID_LOBEAM 				0x10C	//TODO прописать какое-то значение для света
#define LO_BEAM_BYTE				3
#define LO_BEAM_VAL_OFF				0x0

#define CAN_ID_HIBEAM 				0x130
#define HI_BEAM_BYTE				6
#define HI_BEAM_VAL_ON				0x09
#define HI_BEAM_MASK				0x0F

#define CAN_ID_LEFTTURNER 			0x130
#define LEFTTURNER_VAL				0xD2

#define CAN_ID_RIGHTTURNER 			0x130
#define RIGHTTURNER_VAL				0xE2

#define CAN_ID_EMERGENCYLIGHT 		0x130
#define EMERGENCYLIGHT_VAL			0xEA

#define TURNERS_BYTE 				7
#define TURNERS_VAL_OFF				0xCA

#define CAN_ID_STOPLIGHT 			0x00

//проверено CANHACKER
#define CAN_ID_HEATER				0x2D0
#define HEATER_BYTE					7
#define HEATER_MASK					0xF0
#define HEATER_VAL_OFF				0xCF
#define HEATER_VAL_1				0xDF
#define HEATER_VAL_2				0xEF


class canHandler {

private:
	status* m_status;       	// Глобальный статус

	void handleEngineOn	(uint8_t* data);
	void handleIgnition(uint8_t* data);
	void handleLoBeam(uint8_t* data);
	void handleHiBeam(uint8_t* data);
	void handleLeftTurner(uint8_t* data);
	void handleRightTurner(uint8_t* data);
	void handleEmergencyLight(uint8_t* data);
	void handleHeater(uint8_t* data);
	void handleStopLight(uint8_t* data);

public:
	canHandler(status* status);


	void peocessCanMessage(CAN_RxHeaderTypeDef* header, uint8_t* data);

};

#endif /* INC_CANHANDLER_H_ */

