/*
 * canHandler.cpp
 *
 *  Created on: Jun 23, 2025
 *      Author: Admin
 */

#include <canHandler.h>

// Конструктор
canHandler::canHandler(status* status)
    : m_status(status) {
    // Пустой конструктор, только инициализация указателя
}

// Основной метод обработки CAN-сообщений
void canHandler::peocessCanMessage(CAN_RxHeaderTypeDef* header, uint8_t* data) {
	// Проверяем ID сообщения и вызываем соответствующий обработчик
	if(header->StdId == CAN_ID_ENGINE_ON)
		handleEngineOn(data);

	if(header->StdId == CAN_ID_IGNITION)
		handleIgnition(data);

	if(header->StdId == CAN_ID_LOBEAM)
		handleLoBeam(data);

	if(header->StdId == CAN_ID_HIBEAM)
		handleHiBeam(data);

	if(header->StdId == CAN_ID_LEFTTURNER)
		handleLeftTurner(data);

	if(header->StdId == CAN_ID_RIGHTTURNER)
		handleRightTurner(data);

	if(header->StdId == CAN_ID_EMERGENCYLIGHT)
		handleEmergencyLight(data);

	if(header->StdId == CAN_ID_HEATER)
		handleHeater(data);

	if(header->StdId == CAN_ID_STOPLIGHT)
		handleStopLight(data);

}


void canHandler::handleEngineOn(uint8_t* data)
{
	if (data[ENGINE_ON_BYTE] == ENGINE_VAL_OFF)
	{
		m_status->setEngineOff();
	}
	else
	{
		m_status->setEngineOn();
	}
}

void canHandler::handleIgnition(uint8_t* data)
{
	m_status->setIgnitionOn();	//просто наличие пакета говорит о включеном зажигании
}

void canHandler::handleLoBeam(uint8_t* data)	//TODO сейчас обрабатывается работающий двигатель вместо ближнего света
{
	if (data[ENGINE_ON_BYTE] == ENGINE_VAL_OFF)
	{
		m_status->setLoBeamOff();
	}
	else
	{
		m_status->setLoBeamOn();
	}
}

void canHandler::handleHiBeam(uint8_t* data)
{
	if ((data[HI_BEAM_BYTE] & HI_BEAM_MASK) == HI_BEAM_VAL_ON)
	{
		m_status->setHiBeamOn();
	}
	else
	{
		m_status->setHiBeamOff();
	}
}

void canHandler::handleLeftTurner(uint8_t* data)
{
	if (data[TURNERS_BYTE] == LEFTTURNER_VAL)
	{
		m_status->setLeftTurnerOn();
	}
	else
	{
		m_status->setLeftTurnerOff();
	}
}

void canHandler::handleRightTurner(uint8_t* data)
{
	if (data[TURNERS_BYTE] == RIGHTTURNER_VAL)
	{
		m_status->setRightTurnerOn();
	}
	else
	{
		m_status->setRightTurnerOff();
	}
}

void canHandler::handleEmergencyLight(uint8_t* data)
{
	if (data[TURNERS_BYTE] == EMERGENCYLIGHT_VAL)
		{
			m_status->setEmergencyLightOn();
		}
		else
		{
			m_status->setEmergencyLightOff();
		}
}

void canHandler::handleHeater(uint8_t* data)
{
	if (data[HEATER_BYTE] == HEATER_VAL_OFF)
	{
		m_status->setHeaterOff();
	}
	else if (data[HEATER_BYTE] == HEATER_VAL_1)
	{
		m_status->setHeaterOn(1);
	}
	else if (data[HEATER_BYTE] == HEATER_VAL_2)
	{
		m_status->setHeaterOn(2);
	}
}

void canHandler::handleStopLight(uint8_t* data)
{
	// Заглушка - будет реализована позже
}
