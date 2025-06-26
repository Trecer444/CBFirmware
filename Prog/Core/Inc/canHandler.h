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
#define CAN_ID_ENGINE_ON 			0x10C
#define CAN_ID_IGNITION				0x130 //или 2D0, надо проверить
#define CAN_ID_LOBEAM 				0x00
#define CAN_ID_HIBEAM 				0x130
#define CAN_ID_LEFTTURNER 			0x00
#define CAN_ID_RIGHTTURNER 			0x00
#define CAN_ID_EMERGENCYLIGHT 		0x00
#define CAN_ID_HEATER				0x2D0
#define CAN_ID_STOPLIGHT 			0x00

//проверено CANHACKER
#define HEATER_BYTE			7
#define HEATER_MASK			0xF0
#define HEATER_VAL_OFF		0xCF
#define HEATER_VAL_1		0xDF
#define HEATER_VAL_2		0xEF

#define IGNITION_BYTE		0
#define IGNITION_VAL_ON 	0x77
#define IGNITION_VAL_OFF 	0x73

#define HI_BEAM_BYTE		6
#define HI_BEAM_VAL_ON		0x09
#define HI_BEAM_MASK		0x0F


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

