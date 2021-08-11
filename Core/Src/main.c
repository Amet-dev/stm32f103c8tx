
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskGpio */
osThreadId_t myTaskGpioHandle;
const osThreadAttr_t myTaskGpio_attributes = {
  .name = "myTaskGpio",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
//ПЕРЕМЕННЫЕ

modbusHandler_t ModbusH;
uint16_t ModbusDATA[10];
uint16_t shim=0;
uint16_t counter_pwm=600;
const uint16_t  i_nakal_min=3000,
				i4=0,
				u_dc_min=2980,
				u_dc_max=3350,
				t1=1860;
_Bool adc_enable=false;
//регистр модбас код ошибки
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //ИНИЦИАЛИЗАЦИЯ
  ModbusH.uiModbusType = SLAVE_RTU;
  ModbusH.port =  &huart1;
  ModbusH.u8id = 01; //Modbus slave ID
  ModbusH.u16timeOut = 1000;
  ModbusH.EN_Port = NULL;
  ModbusH.u32overTime = 0;
  ModbusH.au16regs = ModbusDATA;
  ModbusH.u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
  //Initialize Modbus library
  ModbusInit(&ModbusH);
  //Start capturing traffic on serial Port
  ModbusStart(&ModbusH);

  HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
  HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_RESET);

  HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);
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

  /* creation of myTaskGpio */
  myTaskGpioHandle = osThreadNew(StartTask02, NULL, &myTaskGpio_attributes);

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
  while (1)
  {
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

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim4.Init.Period = 599;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 600;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO7_GPIO_Port, DO7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DO1_Pin|DO2_Pin|DO3_Pin|DO6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DI1_Pin DI2_Pin DI3_Pin DI4_Pin
                           DI5_Pin DI6_Pin */
  GPIO_InitStruct.Pin = DI1_Pin|DI2_Pin|DI3_Pin|DI4_Pin
                          |DI5_Pin|DI6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DI7_Pin */
  GPIO_InitStruct.Pin = DI7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DI7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DO7_Pin */
  GPIO_InitStruct.Pin = DO7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DO1_Pin DO2_Pin DO3_Pin DO6_Pin */
  GPIO_InitStruct.Pin = DO1_Pin|DO2_Pin|DO3_Pin|DO6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
  for(;;)
  {
	  //если поступил запрос с modbus
	  	  if ( ModbusDATA[0] != 0 ) {
	  		  // проверка введенной команды и установка пинов
	  	  switch (ModbusDATA[0])
	  		  {
	  		  case 1: HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
	  				  break;
	  		  case 2: HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_SET);
	  				  break;
	  		  case 3: HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_SET);
	  				  break;
	  		  case 6: HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_SET);
	  				  break;
	  		  case 7: HAL_GPIO_WritePin(DO7_GPIO_Port, DO7_Pin, GPIO_PIN_SET);
	  				  break;
	  		 case 9:
	  			 TIM4->CCR2=counter_pwm;
	  		TIM4->CCR3=0;
	  		HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET);
	  		ModbusDATA[8]=0;
	  		HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_SET);
	  			  	break;

	  		case 11: HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
	  			  	break;
	  		case 12: HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET);
	  			  	break;
	  		case 13: HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_RESET);
	  			  	break;
	  		case 16: HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_RESET);
	  			  	break;
	  		case 17: HAL_GPIO_WritePin(DO7_GPIO_Port, DO7_Pin, GPIO_PIN_RESET);
	  			  	break;
	  		  case 111:
	  				  HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
	  				  HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET);
	  				  HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_RESET);
	  				  HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_RESET);
	  				  HAL_GPIO_WritePin(DO7_GPIO_Port, DO7_Pin, GPIO_PIN_RESET);
	  				  break;
	  		case 11111:
	  					//установка DO2
					  HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_SET);
					  osDelay(8000);
					  HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_SET);
					  osDelay(150);
					  HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
					  adc_enable=true;


	  				  break;
	  		  }
	  		  ModbusDATA[0]=0;
	  		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  	  }
	  	//управление скважностью ШИМ

			  if ( ModbusDATA[8] != shim ) {
				  if ( ModbusDATA[8]>45 ){
/*
					  TIM4->CCR2=(counter_pwm)-(counter_pwm/100)*45;
					  TIM4->CCR3=(counter_pwm/100)*45;
					  shim=45;*/
					  ModbusDATA[8]=45;
				  }
				  else{
					  while (shim<ModbusDATA[8]){
						  shim++;
						  if (shim>45){shim=45;}
						  TIM4->CCR2=(counter_pwm)-(counter_pwm/100)*shim;
						  TIM4->CCR3=(counter_pwm/100)*shim;
						  osDelay(1);
					  }
					  while (shim>ModbusDATA[8]){
						  shim--;
						  TIM4->CCR2=(counter_pwm)-(counter_pwm/100)*shim;
						  TIM4->CCR3=(counter_pwm/100)*shim;
						  osDelay(1);
					  }

				  }

/*
					  TIM4->CCR2=(counter_pwm)-(counter_pwm/100)*ModbusDATA[8];
					  TIM4->CCR3=(counter_pwm/100)*ModbusDATA[8];
*/
			  }
  	  	  	  if ((DI7_GPIO_Port->IDR & DI7_Pin)==0) {
  	  	  	TIM4->CCR2=counter_pwm;
  	  	  		TIM4->CCR3=0;
  	  	  		HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_RESET);
  	  	  		HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET);
  	  	  		ModbusDATA[8]=0;
  	  	  		HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_SET);
			  }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */

/**
* @brief Function implementing the myTaskGpio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  if ((DI7_GPIO_Port->IDR & DI7_Pin)==0){
		  TIM4->CCR2=counter_pwm;
		  	TIM4->CCR3=0;
		  	HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_RESET);
		  	HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET);
		  	ModbusDATA[8]=0;
		  	HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_SET);
	  }


	//установка регистров в соответствии с состоянием пинов
		ModbusDATA[1]=	((DO1_GPIO_Port->IDR & DO1_Pin)!=0)|    //HAL_GPIO_ReadPin(DO1_GPIO_Port, DO1_Pin);
						((DO2_GPIO_Port->IDR & DO2_Pin)!=0)<<1| //HAL_GPIO_ReadPin(DO2_GPIO_Port, DO2_Pin);
						((DO3_GPIO_Port->IDR & DO3_Pin)!=0)<<2| //HAL_GPIO_ReadPin(DO3_GPIO_Port, DO3_Pin);
						//((DO4_GPIO_Port->IDR & DO4_Pin)!=0)<<3| //HAL_GPIO_ReadPin(DO4_GPIO_Port, DO4_Pin);
						//((DO5_GPIO_Port->IDR & DO5_Pin)!=0)<<4| //HAL_GPIO_ReadPin(DO5_GPIO_Port, DO5_Pin);
						((DO6_GPIO_Port->IDR & DO6_Pin)!=0)<<5| //HAL_GPIO_ReadPin(DO6_GPIO_Port, DO6_Pin);
						((DO7_GPIO_Port->IDR & DO7_Pin)!=0)<<6; //HAL_GPIO_ReadPin(DO7_GPIO_Port, DO7_Pin);

		ModbusDATA[2]=	((DI1_GPIO_Port->IDR & DI1_Pin)!=0)| 	//HAL_GPIO_ReadPin(DI1_GPIO_Port, DI1_Pin);
						((DI2_GPIO_Port->IDR & DI2_Pin)!=0)<<1| //HAL_GPIO_ReadPin(DI2_GPIO_Port, DI2_Pin);
						((DI3_GPIO_Port->IDR & DI3_Pin)!=0)<<2| //HAL_GPIO_ReadPin(DI3_GPIO_Port, DI3_Pin);
						((DI4_GPIO_Port->IDR & DI4_Pin)!=0)<<3| //HAL_GPIO_ReadPin(DI4_GPIO_Port, DI4_Pin);
						((DI5_GPIO_Port->IDR & DI5_Pin)!=0)<<4| //HAL_GPIO_ReadPin(DI5_GPIO_Port, DI5_Pin);
						((DI6_GPIO_Port->IDR & DI6_Pin)!=0)<<5| //HAL_GPIO_ReadPin(DI6_GPIO_Port, DI6_Pin);
						((DI7_GPIO_Port->IDR & DI7_Pin)!=0)<<6; //HAL_GPIO_ReadPin(DI7_GPIO_Port, DI7_Pin);

			  //управление частотой
			  /*
			  if ( ModbusDATA[9] != counter_pwm+1)
				  if ModbusDATA[9]>100{
				  counter_pwm+1=ModbusDATA[9];
			  htim4.Init.Period = 599;
				  }
			  HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);
		*/
		//ацп
	//if (adc_enable) {
		HAL_ADCEx_InjectedStart(&hadc1); // запускаем опрос инжект. каналов
		HAL_ADCEx_InjectedPollForConversion(&hadc1,100); // ждём окончания 100мс
		ModbusDATA[3]=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		ModbusDATA[4]=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
		ModbusDATA[5]=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
		HAL_ADCEx_InjectedStart(&hadc2);
		HAL_ADCEx_InjectedPollForConversion(&hadc2,100);
		ModbusDATA[6]=HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
		ModbusDATA[7]=HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
	//}
	osDelay(1);
	/*
    xSemaphoreTake(ModbusH.ModBusSphrHandle , 100);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ModbusH.au16regs[0] & 0x1);
	xSemaphoreGive(ModbusH.ModBusSphrHandle);
    osDelay(200);*/
  }
  /* USER CODE END StartTask02 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
