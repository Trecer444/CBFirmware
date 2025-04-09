/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_VCC_Pin GPIO_PIN_1
#define ADC_VCC_GPIO_Port GPIOA
#define ADC_IS_1_Pin GPIO_PIN_4
#define ADC_IS_1_GPIO_Port GPIOA
#define ADC_IS_2_Pin GPIO_PIN_5
#define ADC_IS_2_GPIO_Port GPIOA
#define ADC_IS_3_Pin GPIO_PIN_6
#define ADC_IS_3_GPIO_Port GPIOA
#define ADC_IS_4_Pin GPIO_PIN_7
#define ADC_IS_4_GPIO_Port GPIOA
#define ADC_IS_5_Pin GPIO_PIN_4
#define ADC_IS_5_GPIO_Port GPIOC
#define ADC_IS_6_Pin GPIO_PIN_5
#define ADC_IS_6_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOB
#define LM_EN_Pin GPIO_PIN_13
#define LM_EN_GPIO_Port GPIOB
#define OUT_1_Pin GPIO_PIN_14
#define OUT_1_GPIO_Port GPIOB
#define OUT_2_Pin GPIO_PIN_15
#define OUT_2_GPIO_Port GPIOB
#define OUT_3_Pin GPIO_PIN_6
#define OUT_3_GPIO_Port GPIOC
#define OUT_4_Pin GPIO_PIN_7
#define OUT_4_GPIO_Port GPIOC
#define OUT_5_Pin GPIO_PIN_8
#define OUT_5_GPIO_Port GPIOC
#define OUT_6_Pin GPIO_PIN_9
#define OUT_6_GPIO_Port GPIOC
#define USB_OTG_VBUS_Pin GPIO_PIN_8
#define USB_OTG_VBUS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
