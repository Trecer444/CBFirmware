/*
 * outputch.h
 *
 *  Created on: Apr 11, 2025
 *      Author: Admin
 */

#ifndef INC_OUTPUTCH_H_
#define INC_OUTPUTCH_H_

#include "stm32f4xx.h"
#include "stdint.h"
#include "settings.h"
#include "stm32f4xx_hal_tim.h"

typedef enum
{
    DISABLECH = 0, //0
    IGNITION, //1
    REGULARCONS, //2
    LOBEAM,
    HIBEAM,
    ANYHEADLIGNT,
    ANYTURNER,
    LEFTTURNER,
    RIGHTTURNER,
    EMERGENCYLIGHT,
    HEATER, //10
    STOPLIGHT
} SIGNALSOURCE;

class OutputCh
{
public:
	OutputCh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, TIM_HandleTypeDef* PWMtim); 	//конструктор при создании определяется только пинами, за которые он отвечает
	void setChSettingsCh(chSpec* settings);				//принимает настройки и сохраняет их внутри класса+
	void activateCh();									//активирует канал+
	void disactivateCh();								//дективирует канал+
	void updateCh();									//обновляет состоние канала (вызывается в main и в прерываниях)
	void buttonTriggered(uint8_t status);				//метод вызывает при приеме КАНовской команды, которая соответствует изсточнику данного канала+
	void updataVoltage(uint16_t voltage);				//в методе передается в качестве аргумента текущее считанное значение напряжения
//	void
	void turnOffCh();
	void turnOnCh();


private:

	void turnOnPWMCh(uint32_t pwmValInner);

	GPIO_TypeDef port;
	uint16_t pin;
	uint8_t isActive = 0, 							//канал включен (активен, если signalSource <> 0)
			outputState = 0,						//состояние выходных транзисторов
			isButtonActiv = 0,						//указывает активна ли кнопка (0 - неактивна, 1 - активна или 1 ст. подогрева, 2 - вторая ступень подогрева)
			isTimerExpired = 0,
			isDelayExpired = 0;
	GPIO_TypeDef* gpioPortMosfet,					//порт транзистора
				 gpioPortCurr;						//порт приема данных с транзистора
	uint16_t gpioPinMosfet, 						//пин транзистора
			 gpioPinCurr,							//пин приема данных с транзистора
			 voltage,								//хранит текущее значение напряжения
			 timerDelayInner,						//ЗАПИСАТЬ ПРИ ВКЛЮЧЕНИИ КАНАЛА
			 timerShutdownInner;					//ЗАПИСАТЬ ПРИ ВКЛЮЧЕНИИ КАНАЛА

	TIM_HandleTypeDef* PWMtim;

	uint16_t
        signalSource,								//нужен т.к. может быть работа без внешнего сигнала (2)
        pwmValue,
        flashType,
        flashCount,
        heater1,
        heater2,
        delayTimerValue,							//задержка выключения
        shutdownTimerValue,							//выключение по таймеру с активным сигналом
        vCutOffValue,
        vAutoEnValue,
        flashFreq,
        currCutOffValue;

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



#endif /* INC_OUTPUTCH_H_ */
