/*
 * settings.cpp
 *
 *  Created on: Apr 10, 2025
 *      Author: Admin
 */

#include "settings.h"

const uint8_t Param::bitLengths[20] = {
    4, 7, 2, 7, 7, 7,
    14, 12, 11, 11, 9, 11,
    1, 1, 1, 1, 1, 1, 1, 1
};

void Param::setDefaultVal()
{
	for (uint8_t i = 0; i < 6; i++)
	{
		this->chSettings[i].signalSource = 0;
		this->chSettings[i].pwmValue = 50;
		this->chSettings[i].flashType = 0;
		this->chSettings[i].flashCount = 5;
		this->chSettings[i].heater1 = 20;
		this->chSettings[i].heater2 = 50;
		this->chSettings[i].delayTimerValue = 0;
		this->chSettings[i].shutdownTimerValue = 300;
		this->chSettings[i].vCutOffValue = 1250;			//сантивольты (делить на 100)
		this->chSettings[i].vAutoEnValue = 1280;        	//сантивольты (делить на 100)
		this->chSettings[i].flashFreq = 200;            	//*100
		this->chSettings[i].currCutOffValue = 100;      	//*100

		this->chSettings[i].engineOn = 0;
		this->chSettings[i].shutdownTimer = 0;
		this->chSettings[i].vCutOff = 0;
		this->chSettings[i].vAutoEn = 0;
		this->chSettings[i].pwm = 0;
		this->chSettings[i].currCutOff = 0;
		this->chSettings[i].delayTimer = 0;
		this->chSettings[i].flash = 0;
	}
}

Param::Param()
{

	this->setDefaultVal();
}

uint8_t Param::setParam(char *paramString)
{
	uint16_t charLenght = 0;
	while (paramString[charLenght] != '*')
	{
		charLenght++;
	}
	uint16_t arr[20];
	for (uint8_t k = 0; k < 20; k++) arr[k] = 0;
	uint8_t ch = 0, index = 0;
	for (uint16_t i = 0; i < charLenght - 1; i++)
	{
		if (paramString[i]=='*' || paramString[i] == '\n')			//если нашли конец строки то завершаем
			break;
		if (paramString[i] == '$' && i < charLenght - 2)
		{
			i++;
			ch = paramString[i] - '0';
			index = 0;
			i++;
			continue;
		}
		if (paramString[i] == ' ')
		{
			index++;
			continue;
		}
		if (paramString[i] >= '0' && paramString[i] <= '9')
		{
			arr[index] = arr[index]*10 + (paramString[i] - '0');
			continue;
		}
		if (paramString[i] == ';')
		{
			this->chSettings[ch].signalSource = arr[0];
			this->chSettings[ch].pwmValue = arr[1];
			this->chSettings[ch].flashType = arr[2];
			this->chSettings[ch].flashCount = arr[3];
			this->chSettings[ch].heater1 = arr[4];
			this->chSettings[ch].heater2 = arr[5];
			this->chSettings[ch].delayTimerValue = arr[6];
			this->chSettings[ch].shutdownTimerValue = arr[7];
			this->chSettings[ch].vCutOffValue = arr[8];			//сантивольты (делить на 100)
			this->chSettings[ch].vAutoEnValue = arr[9];        	//сантивольты (делить на 100)
			this->chSettings[ch].flashFreq = arr[10];            	//*100
			this->chSettings[ch].currCutOffValue = arr[11];      	//*100

			this->chSettings[ch].engineOn = arr[12];
			this->chSettings[ch].shutdownTimer = arr[13];
			this->chSettings[ch].vCutOff = arr[14];
			this->chSettings[ch].vAutoEn = arr[15];
			this->chSettings[ch].pwm = arr[16];
			this->chSettings[ch].currCutOff = arr[17];
			this->chSettings[ch].delayTimer = arr[18];
			this->chSettings[ch].flash = arr[19];
			for (uint8_t k = 0; k < 20; k++) arr[k] = 0;
		}

	}
	return 0;
}

uint8_t Param::getSourceSign(uint8_t index)
{
	return this->chSettings[index].signalSource;

}


void Param::packToAlignedBuffer(uint16_t* buffer)
{
    uint16_t bitIndex = 0;
    uint16_t wordIndex;
    uint8_t bitOffset;
    uint8_t bitsInThisWord;
    uint16_t mask;

    for (uint8_t i = 0; i < 6; ++i)
    {
        for (uint8_t j = 0; j < 20; ++j)
        {
            uint32_t value = 0;

            switch (j)
            {
                case 0:  value = chSettings[i].signalSource;       break;
                case 1:  value = chSettings[i].pwmValue;           break;
                case 2:  value = chSettings[i].flashType;          break;
                case 3:  value = chSettings[i].flashCount;         break;
                case 4:  value = chSettings[i].heater1;            break;
                case 5:  value = chSettings[i].heater2;            break;
                case 6:  value = chSettings[i].delayTimerValue;    break;
                case 7:  value = chSettings[i].shutdownTimerValue; break;
                case 8:  value = chSettings[i].vCutOffValue;       break;
                case 9:  value = chSettings[i].vAutoEnValue;       break;
                case 10: value = chSettings[i].flashFreq;          break;
                case 11: value = chSettings[i].currCutOffValue;    break;
                case 12: value = chSettings[i].engineOn;           break;
                case 13: value = chSettings[i].shutdownTimer;      break;
                case 14: value = chSettings[i].vCutOff;            break;
                case 15: value = chSettings[i].vAutoEn;            break;
                case 16: value = chSettings[i].pwm;                break;
                case 17: value = chSettings[i].currCutOff;         break;
                case 18: value = chSettings[i].delayTimer;         break;
                case 19: value = chSettings[i].flash;              break;
            }

            uint8_t numBits = Param::bitLengths[j];

            while (numBits > 0)
            {
                wordIndex = bitIndex / 16; //в какой элемент массива пишем
                bitOffset = bitIndex % 16; //битовый сдвиг внутри элемента массива
                bitsInThisWord = (16 - bitOffset < numBits) ? (16 - bitOffset) : numBits; //сколько бит сможем записать в это слово

                mask = (1U << bitsInThisWord) - 1; // Например, bitsInThisWord = 5 → mask = 0b00011111

                buffer[wordIndex] &= ~(mask << bitOffset);               // очистка
                buffer[wordIndex] |= ((value & mask) << bitOffset);     // запись

                value >>= bitsInThisWord;	//сдвигаем на уже записанное кол-во бит (если в value меньше 16 бит, то обнулится)
                bitIndex += bitsInThisWord;
                numBits -= bitsInThisWord;
            }
        }
    }
}


void Param::unpackFromAlignedBuffer(const uint16_t* buffer)
{
    uint16_t bitIndex = 0;
    uint16_t wordIndex;
    uint8_t bitOffset;
    uint8_t bitsInThisWord;
    uint16_t mask;

    for (uint8_t i = 0; i < 6; ++i)
    {

        for (uint8_t j = 0; j < 20; ++j)
        {


            uint32_t value = 0;
            uint8_t bitsRead = 0;
            uint8_t numBits = Param::bitLengths[j];

            while (bitsRead < numBits)
            {
                wordIndex = bitIndex / 16;
                bitOffset = bitIndex % 16;
                bitsInThisWord = (16 - bitOffset < numBits - bitsRead) ? (16 - bitOffset) : (numBits - bitsRead);

                mask = (1U << bitsInThisWord) - 1;
                uint16_t bits = (buffer[wordIndex] >> bitOffset) & mask;

                value |= ((uint32_t)bits << bitsRead);

                bitIndex += bitsInThisWord;
                bitsRead += bitsInThisWord;
            }

            switch (j)
            {
                case 0:  chSettings[i].signalSource       = (uint16_t)value; break;
                case 1:  chSettings[i].pwmValue           = (uint16_t)value; break;
                case 2:  chSettings[i].flashType          = (uint16_t)value; break;
                case 3:  chSettings[i].flashCount         = (uint16_t)value; break;
                case 4:  chSettings[i].heater1            = (uint16_t)value; break;
                case 5:  chSettings[i].heater2            = (uint16_t)value; break;
                case 6:  chSettings[i].delayTimerValue    = (uint16_t)value; break;
                case 7:  chSettings[i].shutdownTimerValue = (uint16_t)value; break;
                case 8:  chSettings[i].vCutOffValue       = (uint16_t)value; break;
                case 9:  chSettings[i].vAutoEnValue       = (uint16_t)value; break;
                case 10: chSettings[i].flashFreq          = (uint16_t)value; break;
                case 11: chSettings[i].currCutOffValue    = (uint16_t)value; break;
                case 12: chSettings[i].engineOn           = (uint8_t)value;  break;
                case 13: chSettings[i].shutdownTimer      = (uint8_t)value;  break;
                case 14: chSettings[i].vCutOff            = (uint8_t)value;  break;
                case 15: chSettings[i].vAutoEn            = (uint8_t)value;  break;
                case 16: chSettings[i].pwm                = (uint8_t)value;  break;
                case 17: chSettings[i].currCutOff         = (uint8_t)value;  break;
                case 18: chSettings[i].delayTimer         = (uint8_t)value;  break;
                case 19: chSettings[i].flash              = (uint8_t)value;  break;
            }
        }
    }
}


//
// Проверяет, вся ли область flash свободна (все half-word == 0xFFFF)
//
uint8_t Param::isFlashEmpty(uint32_t address, uint16_t halfwordCount)
{
    for (uint16_t i = 0; i < halfwordCount; ++i)
    {
        if (*((uint16_t*)(address + i * 2)) != 0xFFFF)
            return 0; // хотя бы одно слово занято
    }
    return 1; // всё свободно
}

//
// Вычисляет CRC16 по массиву half-word'ов (uint16_t)
// Используется для проверки целостности записей в Flash
//
uint16_t Param::calculateCRC16(const uint16_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 16; ++j)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

//
// Проверяет, корректны ли все значения в распакованной записи
// Используется в readFromFlash для отбраковки повреждённых записей
//
uint8_t Param::isValidRecord(const uint16_t* buffer)
{
    Param temp;
    temp.unpackFromAlignedBuffer(buffer);

    for (uint8_t i = 0; i < 6; ++i)
    {
        if (temp.chSettings[i].signalSource > 15) return 0;
        if (temp.chSettings[i].pwmValue > 127) return 0;
        if (temp.chSettings[i].flashType > 4) return 0;
        if (temp.chSettings[i].flashCount > 127) return 0;
        if (temp.chSettings[i].heater1 > 127) return 0;
        if (temp.chSettings[i].heater2 > 127) return 0;
        if (temp.chSettings[i].delayTimerValue > 16383) return 0;
        if (temp.chSettings[i].shutdownTimerValue > 4095) return 0;
        if (temp.chSettings[i].vCutOffValue > 2047) return 0;
        if (temp.chSettings[i].vAutoEnValue > 2047) return 0;
        if (temp.chSettings[i].flashFreq > 511) return 0;
        if (temp.chSettings[i].currCutOffValue > 2047) return 0;

        if (temp.chSettings[i].engineOn > 1) return 0;
        if (temp.chSettings[i].shutdownTimer > 1) return 0;
        if (temp.chSettings[i].vCutOff > 1) return 0;
        if (temp.chSettings[i].vAutoEn > 1) return 0;
        if (temp.chSettings[i].pwm > 1) return 0;
        if (temp.chSettings[i].currCutOff > 1) return 0;
        if (temp.chSettings[i].delayTimer > 1) return 0;
        if (temp.chSettings[i].flash > 1) return 0;
    }

    return 1; // все поля валидны
}

//
// Сохраняет текущие параметры chSettings во Flash
// Запись производится в первый свободный блок в секторах 10 или 11
//
uint8_t Param::saveToFlash()
{
    uint16_t buffer[RECORD_SIZE_HALFWORDS] = {0};

    // [0] Версия
    buffer[0] = RECORD_VERSION;

    // [1..N] Параметры
    packToAlignedBuffer(&buffer[1]);

    // [N+1] CRC
    buffer[RECORD_DATA_HALFWORDS] = calculateCRC16(buffer, RECORD_DATA_HALFWORDS);

    HAL_FLASH_Unlock();

    uint32_t currentAddress = FLASH_PAGE1_ADDR;
    uint32_t endAddress = FLASH_PAGE2_ADDR + FLASH_PAGE_SIZE;

    uint8_t written = 0;

    // 🔍 Поиск первого свободного места с шагом 1 halfword
    for (; currentAddress + RECORD_SIZE_HALFWORDS * 2 < endAddress; currentAddress += 2)
    {
        if (isFlashEmpty(currentAddress, RECORD_SIZE_HALFWORDS))
        {
            // 💾 Запись
            for (uint16_t i = 0; i < RECORD_SIZE_HALFWORDS; ++i)
            {
                if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAddress + i * 2, buffer[i]) != HAL_OK)
                {
                    HAL_FLASH_Lock();
                    return 0;
                }
            }
            written = 1;
            break;
        }
    }

    // 🔁 Если не записали — стираем оба сектора и пишем в начало
    if (!written)
    {
        FLASH_EraseInitTypeDef eraseInit;
        uint32_t pageError = 0;

        eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
        eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        eraseInit.Sector = FLASH_SECTOR_10;
        eraseInit.NbSectors = 2;

        if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0;
        }

        currentAddress = FLASH_PAGE1_ADDR;
        for (uint16_t i = 0; i < RECORD_SIZE_HALFWORDS; ++i)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAddress + i * 2, buffer[i]) != HAL_OK)
            {
                HAL_FLASH_Lock();
                return 0;
            }
        }
    }

    HAL_FLASH_Lock();
    return 1;
}

//
// Загружает последнюю валидную (неповреждённую) запись из Flash
// Ищет последнюю запись с валидной CRC и данными
//
uint8_t Param::readFromFlash()
{
    uint32_t currentAddress = FLASH_PAGE1_ADDR;
    uint32_t endAddress = FLASH_PAGE2_ADDR + FLASH_PAGE_SIZE;

    uint32_t lastValidAddress = 0xFFFFFFFF;
    uint16_t buffer[RECORD_SIZE_HALFWORDS];

    // 🔍 Сканируем всю область с шагом 1 halfword
    for (; currentAddress + RECORD_SIZE_HALFWORDS * 2 < endAddress; currentAddress += 2)
    {
        // ⛔ Проверка на пустоту
        if (isFlashEmpty(currentAddress, RECORD_SIZE_HALFWORDS))
        {
            break; // дошли до свободной области
        }

        // 📥 Читаем запись
        for (uint16_t i = 0; i < RECORD_SIZE_HALFWORDS; ++i)
        {
            buffer[i] = *((const volatile uint16_t*)(currentAddress + i * 2));
        }

        // ⚠️ Проверка версии
        if (buffer[0] != RECORD_VERSION)
        {
            continue; // пропускаем несовместимый формат
        }

        // ✅ Проверка CRC
        uint16_t storedCRC = buffer[RECORD_DATA_HALFWORDS];
        uint16_t calcCRC = calculateCRC16(buffer, RECORD_DATA_HALFWORDS);
        if (storedCRC != calcCRC)
        {
            continue;
        }

        // ✅ Проверка содержимого
        if (!isValidRecord(&buffer[1]))
        {
            continue;
        }

        // ✅ Последняя валидная запись
        lastValidAddress = currentAddress;
    }

    if (lastValidAddress == 0xFFFFFFFF)
    {
        return 0; // ничего не найдено
    }

    // 📥 Читаем выбранную запись
    for (uint16_t i = 0; i < RECORD_SIZE_HALFWORDS; ++i)
    {
        buffer[i] = *((const volatile uint16_t*)(lastValidAddress + i * 2));
    }

    // 🎯 Распаковка
    unpackFromAlignedBuffer(&buffer[1]); // пропускаем версию
    return 1;
}

void Param::composeAllParamsString(char *outString)
{
	uint16_t pos = 0;

	for (uint8_t ch = 0; ch < 6; ch++) // Если каналов больше — поменяй 6
	{
		pos += sprintf(&outString[pos], "$%d", ch);

		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].signalSource);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].pwmValue);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].flashType);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].flashCount);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].heater1);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].heater2);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].delayTimerValue);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].shutdownTimerValue);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].vCutOffValue);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].vAutoEnValue);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].flashFreq);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].currCutOffValue);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].engineOn);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].shutdownTimer);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].vCutOff);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].vAutoEn);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].pwm);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].currCutOff);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].delayTimer);
		pos += sprintf(&outString[pos], " %d", this->chSettings[ch].flash);

		pos += sprintf(&outString[pos], ";");
	}

	outString[pos++] = '*';
	outString[pos] = '\0';
}
