/*
 * outputch.cpp
 *
 *  Created on: Apr 11, 2025
 *      Author: Admin
 */


#include "outputch.h"


void OutputCh::disactivateCh()
{
	this->isActive = 0;
	this->updateCh();
}

void OutputCh::activateCh()
{
	this->isActive = 1;
	this->updateCh();
}

void OutputCh::setChSettingsCh(chSpec* settings)
{
	signalSource = settings->signalSource;
	pwmValue = settings->pwmValue;
	flashType = settings->flashType;
	flashCount = settings->flashCount;
	heater1 = settings->heater1;
	heater2 = settings->heater2;
	delayTimerValue = settings->delayTimerValue * 1000;			//перевод из мс в с
	shutdownTimerValue = settings->shutdownTimerValue * 1000;	//перевод из мс в с
	vCutOffValue = settings->vCutOffValue;
	vAutoEnValue = settings->vAutoEnValue;
	flashFreq = settings->flashFreq;
	currCutOffValue = settings->currCutOffValue;
	engineOn = settings->engineOn;
	shutdownTimer = settings->shutdownTimer;
	vCutOff = settings->vCutOff;
	vAutoEn = settings->vAutoEn;
	pwm = settings->pwm;
	currCutOff = settings->currCutOff;
	delayTimer = settings->delayTimer;
	flash = settings->flash;
	if (signalSource)
		isActive = 1;
	if (signalSource == REGULARCONS || signalSource == HEATER) //в этих режимах таймер задержки выключения неактивен
	{
		delayTimer = 0;
	}
	if (signalSource == REGULARCONS) 		//в этом режиме "только при раб. двиг" неактивно
	{
		engineOn = 0;
	}
	this->updateCh();
}

uint8_t OutputCh::isButtonTriggered()
{
	switch (signalSource)
	{
	case DISABLECH:
		return 0;
		break;
	case IGNITION:
		return statusInstance->getIgnitionStatus();
		break;

	case REGULARCONS:
		return 1;
		break;

	case LOBEAM:
		return statusInstance->getLoBeam();
		break;

	case HIBEAM:
		return statusInstance->getHiBeam();
		break;

	case ANYHEADLIGNT:
		return statusInstance->getAnyBeam();
		break;

	case ANYTURNER:
		return statusInstance->getAnyTurner();
		break;

	case LEFTTURNER:
		return statusInstance->getLeftTurner();
		break;

	case RIGHTTURNER:
		return statusInstance->getRightTurner();
		break;

	case EMERGENCYLIGHT:
		return statusInstance->getEmergencyLight();
		break;

	case HEATER:
		return statusInstance->getHeater();
		break;

	case STOPLIGHT:
		return statusInstance->getStopLight();
		break;
	}
	errorHandler(IS_BUT_TRIG_ERROR);
	return 0;
}

void OutputCh::updataVoltage(uint16_t voltage)
{
	this->voltage = voltage;

	this->updateCh();
}

void OutputCh::updateCh()
{

	if (isButtonActiv != isButtonTriggered())	//если изменилось состояние входного сигнала (кнопки)
	{
		if (isButtonTriggered())				//если кнопка включилась (1 или 2 для подогрева)
		{
			timerShutdownInner = HAL_GetTick();
			isButtonActiv = isButtonTriggered();
			if (checkChCanBeOn())
			{
				turnOnCh();
			}
		}
		else									//если кнопка выключилась ( 0 )
		{
			timerDelayInner = HAL_GetTick();
			isButtonActiv = 0;
			if (!delayTimer)
				turnOffCh();
		}
	}

	//если состояние кнопки не менялось, то просто проверяем может ли канал работать (если состояние менялось, то в верхенем условии мы уже запишем значения таймоеров)
	if (checkChCanBeOn())
	{
		turnOnCh();
	}
	else
	{
		turnOffCh();
	}

}

void OutputCh::turnOffCh()
{

	if (outputState == 0)
		return;

	switch (gpioPinMosfet)
	{
	case 6:
		TIM8->CCR1 = 0;
		break;
	case 7:
		TIM8->CCR2 = 0;
		break;
	case 8:
		TIM8->CCR3 = 0;
		break;
	case 9:
		TIM8->CCR4 = 0;
		break;
	case 14:
		TIM12->CCR1 = 0;
		break;
	case 15:
		TIM12->CCR2 = 0;
		break;
	default:
		errorHandler(TURN_OFF_ERROR);
	}

	outputState = 0;
}

void OutputCh::turnOnCh()
{
//	if (outputState) TODO прописать так, чтобы нормально работало с настройкой каналов
//		return;

	if (signalSource == HEATER)
	{
		if (isButtonActiv == 1)
		{
			switch (gpioPinMosfet)
			{
			case 6:
				TIM8->CCR1 = PwmValCalculator(heater1);
				break;
			case 7:
				TIM8->CCR2 = PwmValCalculator(heater1);
				break;
			case 8:
				TIM8->CCR3 = PwmValCalculator(heater1);
				break;
			case 9:
				TIM8->CCR4 = PwmValCalculator(heater1);
				break;
			case 14:
				TIM12->CCR1 = PwmValCalculator(heater1);
				break;
			case 15:
				TIM12->CCR2 = PwmValCalculator(heater1);
				break;
			default:
				errorHandler(TURN_ON_ERROR);
			}
			outputState = 1;
			return;
		}
		else if (isButtonActiv == 2)
		{
			switch (gpioPinMosfet)
			{
			case 6:
				TIM8->CCR1 = PwmValCalculator(heater2);
				break;
			case 7:
				TIM8->CCR2 = PwmValCalculator(heater2);
				break;
			case 8:
				TIM8->CCR3 = PwmValCalculator(heater2);
				break;
			case 9:
				TIM8->CCR4 = PwmValCalculator(heater2);
				break;
			case 14:
				TIM12->CCR1 = PwmValCalculator(heater2);
				break;
			case 15:
				TIM12->CCR2 = PwmValCalculator(heater2);
				break;
			default:
				errorHandler(TURN_ON_ERROR);
			}
			outputState = 1;
			return;
		}
		else
		{
			errorHandler(TURN_ON_HEATER_ERROR);
			return;
		}
	}

	if (pwm)
	{
		switch (gpioPinMosfet)
		{
		case 6:
			TIM8->CCR1 = PwmValCalculator(pwmValue);
			break;
		case 7:
			TIM8->CCR2 = PwmValCalculator(pwmValue);
			break;
		case 8:
			TIM8->CCR3 = PwmValCalculator(pwmValue);
			break;
		case 9:
			TIM8->CCR4 = PwmValCalculator(pwmValue);
			break;
		case 14:
			TIM12->CCR1 = PwmValCalculator(pwmValue);
			break;
		case 15:
			TIM12->CCR2 = PwmValCalculator(pwmValue);
			break;
		default:
			errorHandler(TURN_ON_ERROR);
		}

	}
	else
	{
		switch (gpioPinMosfet)
		{
		case 6:
			TIM8->CCR1 = MAX_PWM_VAL;
			break;
		case 7:
			TIM8->CCR2 = MAX_PWM_VAL;
			break;
		case 8:
			TIM8->CCR3 = MAX_PWM_VAL;
			break;
		case 9:
			TIM8->CCR4 = MAX_PWM_VAL;
			break;
		case 14:
			TIM12->CCR1 = MAX_PWM_VAL;
			break;
		case 15:
			TIM12->CCR2 = MAX_PWM_VAL;
			break;
		default:
			errorHandler(TURN_ON_ERROR);
		}
	}

	outputState = 1;
}

uint8_t OutputCh::checkChCanBeOn()
{
	if (isActive == 0)
	{
		return 0;
	}

	//если включена опция "только при раб. двиг." и двигатель заглушен
	if (engineOn && !statusInstance->getEngineStatus())
	{
		return 0;
	}

	//если активен контроль напряжения канала и напряжение просело
	if (vCutOff && voltage < vCutOffValue)
	{
		return 0;
	}

	// включение на время (shutdown записывается при включении), через shutdowntimerVALUE нужно выключить
	if (shutdownTimer && timerShutdownInner && (HAL_GetTick() - timerShutdownInner > shutdownTimerValue))
	{
		return 0;
	}

	//если входной сигнал отрицительный
	if (isButtonActiv == 0)
	{
		if(delayTimer && timerDelayInner && (HAL_GetTick() - timerDelayInner < delayTimerValue))	//если установлен таймер задержки и произошло событие запуска таймера задержки и время не вышло
		{
			return 1;
		}
		else //если нет таймера задержки выключения или он вышел то просто выключаем
		{
			return 0;
		}
	}


	//если нажата кнопка
	if (isButtonActiv)
	{

		if (shutdownTimer && timerShutdownInner && (HAL_GetTick() - timerShutdownInner > shutdownTimerValue)) //если включен таймер выключения и время подошло
		{
			return 0;
		}

		else
		{
			return 1;
		}
	}


	return 0; //если ни одно из условий почему-то не выполнилось
}

OutputCh::OutputCh(GPIO_TypeDef* GPIOx, uint8_t GPIO_Pin, TIM_HandleTypeDef* PWMtim, status *data)
	: statusInstance(data), gpioPortMosfet(GPIOx), gpioPinMosfet(GPIO_Pin), PWMtim(PWMtim)
{
//	this->gpioPortMosfet = GPIOx;
//	this->gpioPinMosfet = GPIO_Pin;
//	this->PWMtim = PWMtim;
}

uint16_t OutputCh::PwmValCalculator(uint8_t val)
{
	if(val > 100)
		{
			val = 100;
		}
	return (uint16_t)(MAX_PWM_VAL * val / 100);
}


void OutputCh::errorHandler(ERROR_TYPEDEF err)
{

}

