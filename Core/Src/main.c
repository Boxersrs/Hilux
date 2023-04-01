/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
struct tps_rpm {
  uint8_t tmp;
  uint16_t rpm;
  uint16_t count;
  int procent;
  uint16_t adc;
  double coef;
} tps_rpm;
#include "SMA_filter_lib.h"
uint16_t SMA_Filter_Buffer_1[SMA_FILTER_ORDER] = {
    0,
};
uint16_t SMA_Filter_Buffer_2[SMA_FILTER_ORDER] = {
    0,
};
uint16_t ADC_SMA_Data[2] = {
    0,
};
uint16_t ADC_RAW_Data[2] = { 0, };
uint8_t transmitUART[30];
uint8_t AC = 0;
uint8_t debug = 0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
// Скалинг датчика температуры
// Первое значение - температура в градусах+40
// второе значение - ADC, которое считается по формуле
double scale_temp[2][28] = {
    {160, 150, 140, 134, 133, 132, 131, 130, 129, 128, 126,  124,  122,  121,  118, 110, 102,  96,   88,   80,   72,   64,   56,   48,   40,   32,   24,    0},
    {200, 250, 265, 295, 310, 325, 330, 325, 331, 362, 377,  400,  415,  419,  470, 615, 765, 915, 1065, 1215, 1415, 1615, 1695, 1815, 1965, 2115, 2265, 2650}};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
uint32_t myTask02Buffer[ 128 ];
osStaticThreadDef_t myTask02ControlBlock;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .cb_mem = &myTask02ControlBlock,
  .cb_size = sizeof(myTask02ControlBlock),
  .stack_mem = &myTask02Buffer[0],
  .stack_size = sizeof(myTask02Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
uint32_t myTask03Buffer[ 128 ];
osStaticThreadDef_t myTask03ControlBlock;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .cb_mem = &myTask03ControlBlock,
  .cb_size = sizeof(myTask03ControlBlock),
  .stack_mem = &myTask03Buffer[0],
  .stack_size = sizeof(myTask03Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
uint32_t myTask04Buffer[ 128 ];
osStaticThreadDef_t myTask04ControlBlock;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .cb_mem = &myTask04ControlBlock,
  .cb_size = sizeof(myTask04ControlBlock),
  .stack_mem = &myTask04Buffer[0],
  .stack_size = sizeof(myTask04Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
uint32_t myTask05Buffer[ 128 ];
osStaticThreadDef_t myTask05ControlBlock;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .cb_mem = &myTask05ControlBlock,
  .cb_size = sizeof(myTask05ControlBlock),
  .stack_mem = &myTask05Buffer[0],
  .stack_size = sizeof(myTask05Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
uint32_t myTask06Buffer[ 128 ];
osStaticThreadDef_t myTask06ControlBlock;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .cb_mem = &myTask06ControlBlock,
  .cb_size = sizeof(myTask06ControlBlock),
  .stack_mem = &myTask06Buffer[0],
  .stack_size = sizeof(myTask06Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
uint32_t myTask07Buffer[ 128 ];
osStaticThreadDef_t myTask07ControlBlock;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .cb_mem = &myTask07ControlBlock,
  .cb_size = sizeof(myTask07ControlBlock),
  .stack_mem = &myTask07Buffer[0],
  .stack_size = sizeof(myTask07Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask08 */
osThreadId_t myTask08Handle;
uint32_t myTask08Buffer[ 128 ];
osStaticThreadDef_t myTask08ControlBlock;
const osThreadAttr_t myTask08_attributes = {
  .name = "myTask08",
  .cb_mem = &myTask08ControlBlock,
  .cb_size = sizeof(myTask08ControlBlock),
  .stack_mem = &myTask08Buffer[0],
  .stack_size = sizeof(myTask08Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);
void StartTask08(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef rxHeader;  // CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader;  // CAN Bus Receive Header
uint8_t canRx[8];              // CAN Bus Receive Buffer
CAN_FilterTypeDef canfil;      // CAN Bus Filter
uint32_t canMailbox;           // CAN Bus Mail box variable
uint8_t byte_6_7[2] = {0};

uint8_t can_send_38A[5] = {0x0C, 0, 0, 0, 0};
uint8_t can_send_4C1[8] = {0x01, 0, 0x04, 0x04, 0, 0, 0, 0};
uint8_t can_send_2C4[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t rxbuffer[8];
uint8_t on_off = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /////////////////////////////////////---CAN---/////////////////////////////////
  canfil.FilterBank = 0;
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfil.FilterIdHigh = 0x0000;
  canfil.FilterIdLow = 0x0000;
  canfil.FilterMaskIdHigh = 0;
  canfil.FilterMaskIdLow = 0;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterActivation = ENABLE;
  canfil.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan, &canfil);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(
      &hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
  /////////////////////////////////////---CAN---/////////////////////////////////
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

//  HAL_TIM_Start_IT(&htim1, TIM_CHANNEL_1);
//  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  // uint8_t transmitUART[15];
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(StartTask06, NULL, &myTask06_attributes);

  /* creation of myTask07 */
  myTask07Handle = osThreadNew(StartTask07, NULL, &myTask07_attributes);

  /* creation of myTask08 */
  myTask08Handle = osThreadNew(StartTask08, NULL, &myTask08_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = ENABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 43;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 65535;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 470;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 63488;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Fan2_GPIO_Port, Fan2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OutTacho_GPIO_Port, OutTacho_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Fan1_GPIO_Port, Fan1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : InSpeed_Pin */
  GPIO_InitStruct.Pin = InSpeed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(InSpeed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Fan2_Pin OutTacho_Pin */
  GPIO_InitStruct.Pin = Fan2_Pin|OutTacho_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : InTacho_Pin */
  GPIO_InitStruct.Pin = InTacho_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(InTacho_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Fan1_Pin */
  GPIO_InitStruct.Pin = Fan1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Fan1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

uint8_t TachoCount = 1;
uint8_t delitel_tacho = 2;  // Делитель тахометра - 2 - было четыре цилиндра, стало 6
uint8_t SpeedCount = 0;
uint8_t delitel = 10;  // Делитель спидометра
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//  if (htim->Instance == TIM2) {
//    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
//      tps_rpm.count = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
//      TIM2->CNT = 0;
//    }
//  }
//}

void HAL_GPIO_EXTI_Callback(uint16_t InSpeed) {
//  if (InSpeed == InSpeed_Pin) {
//    if (SpeedCount == delitel){
//    	HAL_GPIO_TogglePin(OutSpeed_GPIO_Port, OutSpeed_Pin);
//    }
//    if (SpeedCount >= delitel){
//    	SpeedCount = 0;
//    }
//    SpeedCount = SpeedCount + 1;
//  } else
	  if (InSpeed == InTacho_Pin) {
	  tps_rpm.count++;
	  if (TachoCount < 5)
	    HAL_GPIO_TogglePin(OutTacho_GPIO_Port, OutTacho_Pin);
	  if (TachoCount == 6) TachoCount = 0;
	  TachoCount = TachoCount + 1;
  } else {
    __NOP();
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;) {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint8_t init = 0;

  /* Infinite loop */
  for (;;) {
		txHeader.IDE = CAN_ID_STD;
		txHeader.RTR = CAN_RTR_DATA;
		txHeader.TransmitGlobalTime = DISABLE;
		if (!init) {
			txHeader.DLC = 8;
			txHeader.StdId = 0x2C4;
			HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_2C4, &canMailbox);
			osDelay(1);
			txHeader.DLC = 8;
			txHeader.StdId = 0x4C1;
			HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_4C1, &canMailbox);
			osDelay(1);
			txHeader.DLC = 5;
			txHeader.StdId = 0x38A;
			HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_38A, &canMailbox);
			osDelay(1);
			txHeader.DLC = 5;
			can_send_38A[0] = 0x08;
			can_send_38A[1] = 0x04;
			txHeader.StdId = 0x38A;
			HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_38A, &canMailbox);
			osDelay(2);
			txHeader.DLC = 8;
			txHeader.StdId = 0x2C4;
			HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_2C4, &canMailbox);
			osDelay(24);
			HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_2C4, &canMailbox);
			osDelay(24);
			HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_2C4, &canMailbox);
			osDelay(24);
			HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_2C4, &canMailbox);
			osDelay(24);
			init++;
		}
		txHeader.DLC = 8;
		txHeader.StdId = 0x2C4;
		HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_2C4, &canMailbox);
		osDelay(24);
		if (tps_rpm.count > 100) {
			can_send_2C4[0] = 0x03;
			can_send_2C4[1] = 0xc8;
		} else {
			can_send_2C4[0] = 0;
			can_send_2C4[1] = 0;
		}
	}
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */

// ADC read an FAN remote

double calc_temp(double adc) {  // �?нтерполяция температуры
	uint8_t size = sizeof(scale_temp[0]) / sizeof(double);
	double second = adc;
	double first = 0;
	uint8_t i = 0;

	/* Поиск первого числа */
	for (i = 0; i < size; i++) {
		if (scale_temp[1][i] == second) {
			first = scale_temp[0][i];
			break;
		} else if (scale_temp[1][i] > second) {
			double x1 = scale_temp[1][i - 1];
			double x2 = scale_temp[1][i];
			double y1 = scale_temp[0][i - 1];
			double y2 = scale_temp[0][i];
			first = y1 + ((y2 - y1) / (x2 - x1)) * (second - x1);
			break;
		}
	}
	return first;
}
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  uint8_t x = 0;
  /* Infinite loop */
	for (;;) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		ADC_RAW_Data[0] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		if (x > 40) {
			tps_rpm.tmp = calc_temp((double) ADC_SMA_Data[0]);
			tps_rpm.procent = tps_rpm.tmp - 67;
			if (tps_rpm.procent < 1){
				TIM3->CCR3 = 0;
				debug = 1;
			}
			else {
				TIM3->CCR3 = 63488 / 100 * tps_rpm.procent;
				debug = 2;
			}
		} else {
			x++;
			TIM3->CCR3 = 0;
			debug = 3;
		}

		if (tps_rpm.tmp - 40 > 93)
			on_off = 1; // Включение вентилятора
		if (tps_rpm.tmp - 40 < 90)
			on_off = 0;  // Выключение вентилятора
		if (byte_6_7[0] == 0xFF || byte_6_7[0] == 0xFF)
			on_off = 1; // Включение вентилятора по команде кондиционера
		osDelay(50);
	}
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRx) != HAL_OK) {
		// Reception Error //
		Error_Handler();
	}
	if ((rxHeader.StdId == 0x380) && (rxHeader.IDE == CAN_ID_STD)) {
		if (canRx[0] == 0x80)
			AC = 1;
		if (canRx[0] == 0)
			AC = 0;
	}
}
/////////////////////////////////////---ReadCAN---/////////////////////////////////
/**
 * @brief Function implementing the myTask04 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for (;;) {
		HAL_UART_Transmit(&huart1, transmitUART,
				sprintf((char*) transmitUART, "%d\t%d\t%d\n",
						tps_rpm.tmp - 40, ADC_SMA_Data[0], debug), 0xfff);

		if (AC == 1 || on_off == 1) {
			HAL_GPIO_WritePin(Fan1_GPIO_Port, Fan1_Pin, SET);
			TIM2->CCR2 = 30000;
		} else {
			HAL_GPIO_WritePin(Fan1_GPIO_Port, Fan1_Pin, RESET);
			TIM2->CCR2 = 65535;
		}
		osDelay(500);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
 * @brief Function implementing the myTask05 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */

//  double buffer = 0;

  /* Infinite loop */
  for (;;) {

			ADC_SMA_Data[0] = SMA_FILTER_Get_Value(SMA_Filter_Buffer_1, &ADC_RAW_Data[0]);

    osDelay(30);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */

  /* Infinite loop */
  for(;;)
  {
		txHeader.IDE = CAN_ID_STD;
		txHeader.RTR = CAN_RTR_DATA;
		txHeader.TransmitGlobalTime = DISABLE;
		txHeader.DLC = 7;
		uint8_t can_send_3B4[7] = { 0, 0x08, 0, tps_rpm.tmp, 0, 0, 0 };
		txHeader.StdId = 0x3B4;
		HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_3B4, &canMailbox);
		osDelay(1024);
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */

  /* Infinite loop */
  for(;;)
  {
		txHeader.IDE = CAN_ID_STD;
		txHeader.RTR = CAN_RTR_DATA;
		txHeader.TransmitGlobalTime = DISABLE;
		txHeader.DLC = 5;
		txHeader.StdId = 0x38A;
		if (AC == 1) {
			can_send_38A[0] = 0x00;
		} else {
			can_send_38A[0] = 0x08;
		}

		HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_38A, &canMailbox);
		osDelay(1152);
  }
  /* USER CODE END StartTask07 */
}

/* USER CODE BEGIN Header_StartTask08 */
/**
* @brief Function implementing the myTask08 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask08 */
void StartTask08(void *argument)
{
  /* USER CODE BEGIN StartTask08 */

  /* Infinite loop */
  for(;;)
  {
		txHeader.IDE = CAN_ID_STD;
		txHeader.RTR = CAN_RTR_DATA;
		txHeader.TransmitGlobalTime = DISABLE;
		txHeader.DLC = 8;
		txHeader.StdId = 0x4C1;
		HAL_CAN_AddTxMessage(&hcan, &txHeader, can_send_4C1, &canMailbox);
		osDelay(910);
  }
  /* USER CODE END StartTask08 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM2) {
    tps_rpm.count = 0;
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
