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

// Пустые обработчики сообщений (будут заполнены позже)
void canHandler::handleEngineOn(uint8_t* data)
{
	// Заглушка - будет реализована позже
}

void canHandler::handleIgnition(uint8_t* data)
{
	if (data[IGNITION_BYTE] == IGNITION_VAL_ON)
	{
		m_status->setIgnitionOn();
	}
	else
	{
		m_status->setIgnitionOff();
	}
}

void canHandler::handleLoBeam(uint8_t* data)
{
	// Заглушка - будет реализована позже
}

void canHandler::handleHiBeam(uint8_t* data)
{
	// Заглушка - будет реализована позже
}

void canHandler::handleLeftTurner(uint8_t* data)
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

void canHandler::handleRightTurner(uint8_t* data)
{
	// Заглушка - будет реализована позже
}

void canHandler::handleEmergencyLight(uint8_t* data)
{
	// Заглушка - будет реализована позже
}

void canHandler::handleHeater(uint8_t* data)
{
	if ((data[HEATER_BYTE] & HEATER_MASK) == HEATER_VAL_OFF)
	{
		m_status->setHeaterOff();
	}
	else if ((data[HEATER_BYTE] & HEATER_MASK) == HEATER_VAL_1)
	{
		m_status->setHeaterOn(1);
	}
	else if ((data[HEATER_BYTE] & HEATER_MASK) == HEATER_VAL_2)
	{
		m_status->setHeaterOn(2);
	}
}

void canHandler::handleStopLight(uint8_t* data)
{
	// Заглушка - будет реализована позже
}
