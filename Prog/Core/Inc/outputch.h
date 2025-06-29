/*
 * outputch.h
 *
 *  Created on: Apr 11, 2025
 *      Author: Admin
 */

#ifndef INC_OUTPUTCH_H_
#define INC_OUTPUTCH_H_

#define MAX_PWM_VAL 399

#include "stm32f4xx.h"
#include "stdint.h"
#include "settings.h"
#include "stm32f4xx_hal_tim.h"
#include "status.h"

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

typedef enum
{
	TURN_OFF_ERROR = 0,
	TURN_ON_ERROR,
	TURN_ON_HEATER_ERROR,
	IS_BUT_TRIG_ERROR
} ERROR_TYPEDEF;

class OutputCh
{
public:
	OutputCh(GPIO_TypeDef* GPIOx, uint8_t GPIO_Pin, TIM_HandleTypeDef* PWMtim, status *data); 	//конструктор при создании определяется только пинами, за которые он отвечает
	void setChSettingsCh(chSpec* settings);				//принимает настройки и сохраняет их внутри класса+
	void activateCh();									//активирует канал+
	void disactivateCh();								//дективирует канал+
	void updateCh();									//обновляет состоние канала (вызывается в main и в прерываниях)
	uint8_t isButtonTriggered();						//метод проверяет поменялось ли значение кнопки
	void updataVoltage(uint16_t voltage);				//в методе передается в качестве аргумента текущее считанное значение напряжения
	void updateCurent(uint16_t current);				//обновляет значение тока через мосфет канала
	void updateStatus(status* _status);					//обновляет значения для канала из статуса мотоцикла
//	void



private:
	GPIO_TypeDef port;
	uint8_t pin;
	status *statusInstance;							//указатель на класс, в котором хранятся (и обновляются) данные о состоянии мотоцикла (каналы, напряжение и т.д.)
	GPIO_TypeDef* gpioPortMosfet,					//порт транзистора
				 gpioPortCurr;						//порт приема данных с транзистора

	uint8_t isActive = 0, 							//канал включен (активен, если signalSource <> 0)
			outputState = 0,						//состояние выходных транзисторов
			isButtonActiv = 0;						//указывает активна ли кнопка (0 - неактивна, 1 - активна или 1 ст. подогрева, 2 - вторая ступень подогрева)

	uint8_t gpioPinMosfet, 							//пин транзистора
			 gpioPinCurr,							//пин приема данных с транзистора
			 voltage,								//хранит текущее значение напряжения
			 current;								//хранит данные о токе через мосфет
	uint32_t
			 timerDelayInner,						//ЗАПИСАТЬ ПРИ ПРОПАДАНИИ СИГНАЛА ВКЛЮЧЕНИЯ КАНАЛА
			 timerShutdownInner,					//ЗАПИСАТЬ ПРИ ВКЛЮЧЕНИИ КАНАЛА
			 delayTimerValue,						//задержка выключения
			 shutdownTimerValue;					//выключение по таймеру с активным сигналом

	TIM_HandleTypeDef* PWMtim;

	uint16_t
        signalSource,								//нужен т.к. может быть работа без внешнего сигнала (2)
        pwmValue,									//значение ШИМ для всего, кроме подогрева
        flashType,
        flashCount,
        heater1,									//значение ШИМ для 1 ступени подогрева
        heater2,									//значение ШИМ для 2 ступени подогрева
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

	void turnOnPWMCh(uint32_t pwmValInner);
	void turnOffCh();
	void turnOnCh();
	uint8_t checkChCanBeOn();
	void errorHandler(ERROR_TYPEDEF err);			//TODO прописать обработчик ошибок
	uint16_t PwmValCalculator(uint8_t val);
};



#endif /* INC_OUTPUTCH_H_ */
