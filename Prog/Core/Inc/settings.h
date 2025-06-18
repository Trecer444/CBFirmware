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
#include "stm32f4xx_hal.h"

// Расположение и размер EEPROM-эмуляции во Flash
#define RECORD_VERSION           0x0001  // Текущая версия структуры записи (ВАЖНО!)
#define FLASH_PAGE1_ADDR         0x080C0000  // Сектор 10
#define FLASH_PAGE2_ADDR         0x080E0000  // Сектор 11
#define FLASH_PAGE_SIZE          0x20000     // 128 KB
#define RECORD_DATA_HALFWORDS    47      // Включая версию и параметры (пример: 1 + 46)
#define RECORD_CRC_HALFWORDS     1
#define RECORD_SIZE_HALFWORDS    (RECORD_DATA_HALFWORDS + RECORD_CRC_HALFWORDS)

uint8_t isFlashEmpty(uint32_t address, uint16_t size);

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



class Param
{
private:
    //параметры платы
	chSpec chSettings[6];
	static const uint8_t bitLengths[20];
    uint8_t isFlashEmpty(uint32_t address, uint16_t halfwordCount);
    uint8_t isValidRecord(const uint16_t* buffer);
    uint16_t calculateCRC16(const uint16_t* data, uint16_t length);
    void packToAlignedBuffer(uint16_t* buffer);
    void unpackFromAlignedBuffer(const uint16_t* buffer);

public:
    Param();
    void setDefaultVal();
    uint8_t setParam(char *paramString, uint16_t stringSize);
    uint8_t setParam(char *paramString);
    uint8_t getParam(char *paramString, uint16_t stringSize);
    uint8_t getParam(char *paramString);
    uint8_t getSourceSign(uint8_t index);
    uint8_t saveToFlash();
    uint8_t readFromFlash();
    void composeAllParamsString(char *outString);

};




#endif /* INC_SETTINGS_H_ */
