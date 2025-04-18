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
	this->signalSource = settings->signalSource;
	this->pwmValue = settings->pwmValue;
	this->flashType = settings->flashType;
	this->flashCount = settings->flashCount;
	this->heater1 = settings->heater1;
	this->heater2 = settings->heater2;
	this->delayTimerValue = settings->delayTimerValue;
	this->shutdownTimerValue = settings->shutdownTimerValue;
	this->vCutOffValue = settings->vCutOffValue;
	this->vAutoEnValue = settings->vAutoEnValue;
	this->flashFreq = settings->flashFreq;
	this->currCutOffValue = settings->currCutOffValue;
	this->engineOn = settings->engineOn;
	this->shutdownTimer = settings->shutdownTimer;
	this->vCutOff = settings->vCutOff;
	this->vAutoEn = settings->vAutoEn;
	this->pwm = settings->pwm;
	this->currCutOff = settings->currCutOff;
	this->delayTimer = settings->delayTimer;
	this->flash = settings->flash;

	this->updateCh();
}

void OutputCh::buttonTriggered(uint8_t status)
{
	this->isButtonActiv = status;
	if (status)
	{
		this->timerShutdownInner = HAL_GetTick();
	}
	else
	{
		this->timerDelayInner = HAL_GetTick();
	}
	this->isActive = 0;
	this->updateCh();
}

void OutputCh::updataVoltage(uint16_t voltage)
{
	this->voltage = voltage;

	this->updateCh();
}

void OutputCh::updateCh()
{
	if (HAL_GetTick() - this->timerDelayInner > this->delayTimerValue && this->delayTimer)	this->isDelayExpired = 1;
	if (HAL_GetTick() - this->shutdownTimer > this->shutdownTimerValue && this->shutdownTimer)	this->isTimerExpired = 1;

	if (this->isActive == 0) //если канал отключен, то просто выключаем выходной мосфет
	{
		if (this->outputState) this->turnOffCh();
		return;
	}
	//==============================================================
	if (this->vCutOff && this->voltage < this->vCutOffValue && this->isActive && this->outputState) //если просело напряжение и активен контроль напр и канал вкл
	{
		this->turnOffCh();
		return;
	}
	//==============================================================
	if (this->isButtonActiv == 0 && this->outputState) //если входной сигнал отрицительный и канал активен
	{
		if(this->delayTimer)							//если установлен таймер задержки
		{
			if (isDelayExpired) //если таймер вышел
			{
				this->turnOffCh();
				return;
			}
		}
		else //если нет таймера задержки выключения то просто выключаем
		{
			this->turnOffCh();
			return;
		}
	}
	//==============================================================


	if (this->isButtonActiv) //если кнопка активна
	{
		if (this->outputState)					//если сейчас включен мосфет
		{
			if (this->shutdownTimer && isTimerExpired) //если включен таймер выключения и время подошло
			{
				this->turnOffCh();
				return;
			}

		}
		else //если кнопка активна, но канал выключен
		{
			if (this->pwm && this->signalSource != HEATER)
			{
				this->turnOnPWMCh (this->pwmValue);
				return;
			}
			if (this->signalSource == HEATER) //если выбран источник сигнала подогрев
			{
				if (this->isButtonActiv == 1)
				{
					this->turnOnPWMCh (this->heater1);
				}
				if (this->isButtonActiv == 2)
				{
					this->turnOnPWMCh (this->heater2);
				}
				return;
			}
			this->turnOnCh();
		}
	}

// TODO: прописать выключение, если активен PWM или подогрев




}

void OutputCh::turnOffCh()
{
	if (this->gpioPinMosfet == 6 || this->gpioPinMosfet == 7 || this->gpioPinMosfet == 8 || this->gpioPinMosfet == 9)
	{
		  TIM8->CCR1 = 0;
		  TIM8->CCR2 = 0;
		  TIM8->CCR3 = 0;
		  TIM8->CCR4 = 0;
	}
	HAL_GPIO_WritePin(this->gpioPortMosfet, this->gpioPinMosfet, GPIO_PIN_RESET);
	this->outputState = 0;
}

void OutputCh::turnOnCh()
{
	if (this->gpioPinMosfet == 6 || this->gpioPinMosfet == 7 || this->gpioPinMosfet == 8 || this->gpioPinMosfet == 9)
	{
		  TIM8->CCR1 = 600;
		  TIM8->CCR2 = 600;
		  TIM8->CCR3 = 600;
		  TIM8->CCR4 = 600;
	}
	HAL_GPIO_WritePin(this->gpioPortMosfet, this->gpioPinMosfet, GPIO_PIN_SET);
	this->outputState = 1;
}


void OutputCh::turnOnPWMCh(uint32_t pwmValInner)
{
	switch (this->pin)
	{
	case 6:
		TIM8->CCR1 = pwmValInner;
		HAL_TIM_PWM_Start(this->PWMtim, TIM_CHANNEL_1);
		this->outputState = 1;
		break;
	case 7:
		TIM8->CCR1 = pwmValInner;
		HAL_TIM_PWM_Start(this->PWMtim, TIM_CHANNEL_2);
		this->outputState = 1;
		break;
	case 8:
		TIM8->CCR1 = pwmValInner;
		HAL_TIM_PWM_Start(this->PWMtim, TIM_CHANNEL_3);
		this->outputState = 1;
		break;
	case 9:
		TIM8->CCR1 = pwmValInner;
		HAL_TIM_PWM_Start(this->PWMtim, TIM_CHANNEL_4);
		this->outputState = 1;
		break;
	default:
		break;
	}
}

OutputCh::OutputCh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, TIM_HandleTypeDef* PWMtim)
{
	this->gpioPortMosfet = GPIOx;
	this->gpioPinMosfet = GPIO_Pin;
	this->PWMtim = PWMtim;
}





