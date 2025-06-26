/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

extern "C"
{
#ifdef __cplusplus
#include "header.h"
#endif
}
#include "settings.h"
#include "outputch.h"
#include "status.h"
#include "canHandler.h"

#include "usbd_cdc_if.h"
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_LENGHT 			512 				//все равно надо вручную прописать в usbd_cdc_if.h
//#define BYTES_AR_SIZE 		338					//размер передаваемой строки в байтах
#define FILTER				2					// = (filtered * FILTER + new_val)/ (FILTER + 1)
#define ADC_CHANNEL_COUNT 	9
#define VCC_COEFF			461
#define VREF_CAL *VREFINT_CAL_ADDR
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef rxCanHeader;
uint8_t rxCanData[8] = {0,};
uint8_t rxData[RX_LENGHT];			//данные с ком-порта;
uint16_t rxIndex = 0;				//указатель на элемент массива rxData для побайтового вычитывания из буфера ком-порта
Param param;
uint16_t 	secFromStart = 0,
			msFromStart = 0,
			updTimer = 0;
uint8_t flagUpdCh = 0;

uint16_t adc_raw[ADC_CHANNEL_COUNT];        // Сырые значения из DMA
ADC_Data_t adc_filtered;					//структура с данными каналов

status currStatus;
canHandler canMsgHandler(&currStatus);

OutputCh* CH[6];					// Массив из 6 указателей на объекты OutputCh

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
  * @brief  Vector base address configuration. It should no longer be at the start of
  *         flash memory but moved forward because the first part of flash is
  *         reserved for the bootloader. Note that this is already done by the
  *         bootloader before starting this program. Unfortunately, function
  *         SystemInit() overwrites this change again.
  * @return none.
  */
static void VectorBase_Config(void)
{
  /* The constant array with vectors of the vector table is declared externally in the
   * c-startup code.
   */
  extern const unsigned long g_pfnVectors[];

  /* Remap the vector table to where the vector table is located for this program. */
  SCB->VTOR = (unsigned long)&g_pfnVectors[0];
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* Configure the vector table base address. */
  VectorBase_Config();

  OutputCh CH0(GPIOB,  14, &htim12, &currStatus);
  OutputCh CH1(GPIOB,  15, &htim12, &currStatus);
  OutputCh CH2(GPIOC,  6, &htim8, &currStatus);
  OutputCh CH3(GPIOC,  7, &htim8, &currStatus);
  OutputCh CH4(GPIOC,  8, &htim8, &currStatus);
  OutputCh CH5(GPIOC,  9, &htim8, &currStatus);
  CH[0] = &CH0;
  CH[1] = &CH1;
  CH[2] = &CH2;
  CH[3] = &CH3;
  CH[4] = &CH4;
  CH[5] = &CH5;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize the user program application. */
  AppInit();



//=============================================CAN CONFIG

  CAN_FilterTypeDef canFilterConfig;
  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = CANFILTER_1 << 5;				//устаенавливаем фильтра на ID сообщений CAN
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = CANFILTER_2 << 5;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);

  canFilterConfig.FilterBank = 1;
  canFilterConfig.FilterIdHigh = CANFILTER_3 << 5;				//устаенавливаем фильтра на ID сообщений CAN
  canFilterConfig.FilterMaskIdHigh = CANFILTER_4 << 5;
  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);

  canFilterConfig.FilterBank = 2;
  canFilterConfig.FilterIdHigh = CANFILTER_5 << 5;				//устаенавливаем фильтра на ID сообщений CAN
  canFilterConfig.FilterMaskIdHigh = CANFILTER_6 << 5;
  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);

  canFilterConfig.FilterBank = 3;
  canFilterConfig.FilterIdHigh = CANFILTER_7 << 5;				//устаенавливаем фильтра на ID сообщений CAN
  canFilterConfig.FilterMaskIdHigh = CANFILTER_8 << 5;
  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);

  canFilterConfig.FilterBank = 4;
  canFilterConfig.FilterIdHigh = CANFILTER_9 << 5;				//устаенавливаем фильтра на ID сообщений CAN
  canFilterConfig.FilterMaskIdHigh = CANFILTER_10 << 5;
  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);

  canFilterConfig.FilterBank = 5;
  canFilterConfig.FilterIdHigh = CANFILTER_11 << 5;				//устаенавливаем фильтра на ID сообщений CAN
  canFilterConfig.FilterMaskIdHigh = CANFILTER_12 << 5;
  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
  //=============================================CAN CONFIG END
  HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, ADC_CHANNEL_COUNT);


  HAL_TIM_Base_Start_IT(&htim2);	//прерывания каждую секунду
  HAL_TIM_Base_Start_IT(&htim3);	//прерывания каждую мс


  TIM8->CCR1 = 0;
  TIM8->CCR2 = 0;
  TIM8->CCR3 = 0;
  TIM8->CCR4 = 0;
  TIM12->CCR1 = 0;
  TIM12->CCR2 = 0;

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);



  __enable_irq();
//  param.saveToFlash();
  param.readFromFlash();


  setChSettings();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    	CH[3]->turnOnCh();
//    	CH[4]->turnOnCh();
//    	CH[5]->turnOnCh();
//    	HAL_Delay(300);
//    	CH[5]->turnOffCh();
//    	CH[4]->turnOffCh();
//    	CH[3]->turnOffCh();
//    	HAL_Delay(300);


    	processUsbCommand();
    	for (int i = 0; i < 6; i++)
    	{
    		CH[i]->updateCh();
    	}
    	HAL_Delay(15);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  BootComCheckActivationRequest();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	CAN_FilterTypeDef sFilterConfig; //хотя бы 1 фильтр должен быть, иначе не заведется
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_SILENT;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /*
   * параметры фильтра, настроенные на прием любого сообщ-я
   */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = CANFILTER_1 << 5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = CANFILTER_2 << 5;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  //sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 359;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 359;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 399;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 359;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 359;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|LM_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin LM_EN_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LM_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_OTG_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_VBUS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//================================================================TIMERS================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)//каждую секунду
	{
		secFromStart++;
        HAL_GPIO_TogglePin(GPIOB, LED_Pin);
	}
	if(htim->Instance == TIM3) //каждую милисекунду
	{
		msFromStart++;
		if (updTimer >=20)
		{
			updTimer = 0;
			flagUpdCh = 1;
		}
		else
		{
			updTimer++;
		}
	}

}

//================================================================ADC================================================================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1)
	{
		adc_filtered.VCC      = (adc_filtered.VCC     * FILTER + (uint16_t)((float)adc_raw[0] * (float)(3300.0f / 4095.0f * 4.615f))) / (FILTER + 1);

		adc_filtered.CH0      = (adc_filtered.CH0     * FILTER + adc_raw[1]) / (FILTER + 1);
		adc_filtered.CH1      = (adc_filtered.CH1     * FILTER + adc_raw[2]) / (FILTER + 1);
		adc_filtered.CH2      = (adc_filtered.CH2     * FILTER + adc_raw[3]) / (FILTER + 1);
		adc_filtered.CH3      = (adc_filtered.CH3     * FILTER + adc_raw[4]) / (FILTER + 1);
		adc_filtered.CH4      = (adc_filtered.CH4     * FILTER + adc_raw[5]) / (FILTER + 1);
		adc_filtered.CH5      = (adc_filtered.CH5     * FILTER + adc_raw[6]) / (FILTER + 1);

		adc_filtered.Temperature  = (adc_filtered.Temperature * FILTER + adc_raw[7]) / (FILTER + 1);
		adc_filtered.Vref         = (adc_filtered.Vref        * FILTER + (uint16_t)((float)(3300.0f * VREF_CAL) / (float)adc_raw[8])) / (FILTER + 1);
	}
}

//================================================================CAN MESSAGES RX================================================================
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	if(HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxCanHeader, rxCanData) == HAL_OK)
	{

		canMsgHandler.peocessCanMessage(&rxCanHeader, rxCanData);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	}
}

//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxCanHeader, rxCanData) == HAL_OK)
//    {
//        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//    }
//}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
//    uint32_t er = HAL_CAN_GetError(hcan);
//    sprintf(trans_str,"ER CAN %lu %08lX", er, er);
//    HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 100);
}

//================================================================USB================================================================
void processUsbCommand()
{
	uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();
	if (bytesAvailable > 0)
	{
		while (bytesAvailable--)
		{
			uint8_t byte;
			if (CDC_ReadRxBuffer_FS(&byte, 1) != USB_CDC_RX_BUFFER_OK)
				break;
			if (rxIndex < RX_LENGHT - 1)
			{
				rxData[rxIndex++] = byte;

				// Конец команды по символу '\n' тут обрабатываем команду
				if (byte == '\n')
				{
					rxData[rxIndex] = '\0'; // Null-terminate
					handleUsbCommand((char*)rxData);
					rxIndex = 0; // Сброс для следующей команды
					for (int i = 0; i < RX_LENGHT; i++)
						rxData[i] = 0;
				}
			}
			else
			{
				// Ошибка: сообщение слишком длинное
				rxIndex = 0;
			}
		}
//				HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
//				uint16_t bytesToRead = bytesAvailable < 8 ? 8 : bytesAvailable;
//				if (CDC_ReadRxBuffer_FS(rxData, bytesToRead) == USB_CDC_RX_BUFFER_OK)
//				{
//					uint8_t recived[] = "RECIVED\n";
//					while (CDC_Transmit_FS(recived, 8) != USBD_OK);
//					param.setParam((char*)rxData);
//					param.saveToFlash();
//
//				}
//				CDC_FlushRxBuffer_FS();
//				HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
	}
}



void setChSettings()
{
	  for (int i = 0; i < 6; i++)
	    {
	  	  CH[i]->setChSettingsCh(param.getChSpec(i));
	    }

}

void handleUsbCommand(char* cmd)
{
	if (strncmp(cmd, "#GET-ParamList", 14) == 0)
	{
		// Запрос строки параметров
		static char paramString[512];
		param.composeAllParamsString(paramString);
		while (CDC_Transmit_FS((uint8_t*)paramString, strlen(paramString)) != USBD_OK);
	}
	else if (strncmp(cmd, "$", 1) == 0)
	{
		// Попытка интерпретации как строки параметров
		param.setParam(cmd);
		param.saveToFlash();
		setChSettings();

		const char ok[] = "OK\n";
		while (CDC_Transmit_FS((uint8_t*)ok, sizeof(ok) - 1) != USBD_OK);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
