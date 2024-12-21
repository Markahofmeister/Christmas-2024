/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*
 * Include board-specific variable mappings
 */
#ifndef CHRISTMAS24_VARS_H_
#include "Christmas24_Vars.h"
#endif

/*
 * Include SPI Flash Memeory Driver
 */
#ifndef W25QXXX_H_
#include "W25Qxxx.h"
#endif

/*
 * Include I2S Amplifier Driver
 */
#ifndef NAU8315YG_H_
#include "NAU8315YG.h"
#endif

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
I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_tx;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */


/*
 * Timers
 */


// Timer to be used for non-blocking delays
TIM_HandleTypeDef *timerDelay = &htim14;

// Timer(s) to PWM display
TIM_HandleTypeDef *timerPWM[4] = {&htim1, &htim1, &htim3, &htim3};

// Timer to time LED changes
TIM_HandleTypeDef *timerLED = &htim16;

// Timer to perform dimming
TIM_HandleTypeDef *timerDim = &htim17;


/*
 * SPI Flash memory struct
 */
W25Q spiFlash;

/*
 * I2S amplifier struct
 */
NAU8315YG i2sAmp;

/*
 * DMA buffers & flags
 */

// Two buffers to ping-pong in between for SPI RX
uint8_t spiRxBuff[BUFFER_SIZE];

// Single circular output buffer to TX I2S audio data
uint16_t i2sTxBuff[BUFFER_SIZE * 2];
//uint16_t i2sTxBuff[BUFFER_SIZE * 2];

// Shall jam be started?
bool jamStart = false;

// Are we already jamming?
bool inJam = false;

// Variable to keep track of where to read audio data from in memory
uint32_t flashReadAddr = initialMemoryOffset;

// Used to keep track of where the song is and trigger LED changes.
uint32_t songIndex = 0;

uint8_t audioEn = 1;

uint32_t dimIndex = 0;

uint8_t dimNum = 0;

uint8_t systemPWM = 50;

uint32_t timerChangeIndex[3] = {1034, 0x00, 0x00};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

void flashInit(void);

/*
 * Displays non-critical error on debug LED and continues functionality
 */
void dispFault();

/*
 * Displays Critical Error on clock and halts all functionality
 */
void dispFailure();

/*
 * Begins double-buffered DMA streams to provide audio output
 */
void startAudioStream(void);

/*
 * Halts double-buffered DMA streams
 */
void stopAudioStream(void);


// Fills I2S tx buffer at a given array index offset
void fillTxBuffer(uint16_t offset);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	  MX_DMA_Init();
	  MX_I2S1_Init();
	  MX_SPI2_Init();
	  MX_TIM1_Init();
	  MX_TIM3_Init();
	  MX_TIM14_Init();
	  MX_TIM16_Init();
	  MX_TIM17_Init();
	  /* USER CODE BEGIN 2 */

	  flashInit();


		// Init Memory Chip
		uint8_t initRet = W25Q_Init(&spiFlash, nCSPort, nWPPort, nHOLDPort,
				 nCSPin, nWPPin, nHOLDPin, &hspi2, spiFlash_devID, spiFlash_isQuadChip, spiFlash_driveStrength);

		// Enter error loop if there's an error in initialization
		if( (initRet == 1) || (initRet == 7) ) {
			/* Critical Errors:
			* 1 = Failure to release chip from power down
			* 7 = Failure to disable write protection
			*/
			dispFailure();
		}
		else if ( ((initRet >= 2) && (initRet <= 6)) || (initRet == 8) ) {
			/*
			* Non-critical Errors:
			* 2 = Failure to reset chip
			* 3,6,8 = Failure to read status registers
			* 4 = Failure to set driver strength
			* 5 = Failure to read device ID
			*/
			dispFault();
		}
		else if(initRet == 0) {
			// initRet = 0 = all is well
			__NOP();
		}


		// Init i2s amplifier
		NAU8315YG_Init(&i2sAmp, &hi2s1, i2sAmp_enablePort, i2sAmp_enablePin);

		// Increase SPI clk driver power
		GPIOA->OSPEEDR |= 0b11;

		timerChangeIndex[1] = timerChangeIndex[0] + 72;
		timerChangeIndex[2] = timerChangeIndex[1] + 64;

	  /* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */


	while (1)
	{

		if(jamStart && !inJam) {
			startAudioStream();
		}

	  /* USER CODE END WHILE */


	  /* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 800-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 800-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 244*4;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 127;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 50674;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 4-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 63999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CHIP_SELECT_Pin|MEM_nWP_Pin|MEM_nHOLD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED1_Pin|LED2_Pin|LED10_Pin
                          |LED11_Pin|LED12_Pin|LED13_Pin|LED14_Pin
                          |LED15_Pin|LED3_Pin|LED4_Pin|LED5_Pin
                          |LED6_Pin|LED7_Pin|LED8_Pin|LED9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AMP_ENABLE_GPIO_Port, AMP_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI_CHIP_SELECT_Pin MEM_nWP_Pin MEM_nHOLD_Pin */
  GPIO_InitStruct.Pin = SPI_CHIP_SELECT_Pin|MEM_nWP_Pin|MEM_nHOLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin LED10_Pin
                           LED11_Pin LED12_Pin LED13_Pin LED14_Pin
                           LED15_Pin LED3_Pin LED4_Pin LED5_Pin
                           LED6_Pin LED7_Pin LED8_Pin LED9_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin|LED10_Pin
                          |LED11_Pin|LED12_Pin|LED13_Pin|LED14_Pin
                          |LED15_Pin|LED3_Pin|LED4_Pin|LED5_Pin
                          |LED6_Pin|LED7_Pin|LED8_Pin|LED9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AMP_ENABLE_Pin */
  GPIO_InitStruct.Pin = AMP_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AMP_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : JAM_BUTTON_EXTI_Pin */
  GPIO_InitStruct.Pin = JAM_BUTTON_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(JAM_BUTTON_EXTI_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void flashInit(void) {

	uint8_t initPWM = systemPWM;
	uint8_t delay = 1;

	__HAL_TIM_SET_COMPARE(timerPWM[0], PWMChannels[0], initPWM);
	HAL_TIM_PWM_Start(timerPWM[0], PWMChannels[0]);
	__HAL_TIM_SET_COMPARE(timerPWM[1], PWMChannels[1], initPWM);
	HAL_TIM_PWM_Start(timerPWM[1], PWMChannels[1]);
	__HAL_TIM_SET_COMPARE(timerPWM[2], PWMChannels[2], initPWM);
	HAL_TIM_PWM_Start(timerPWM[2], PWMChannels[2]);
	__HAL_TIM_SET_COMPARE(timerPWM[3], PWMChannels[3], initPWM);
	HAL_TIM_PWM_Start(timerPWM[3], PWMChannels[3]);

	GPIO_TypeDef *GPIOPort = GPIOB;

	const uint16_t GPIOPins[16] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3,
									GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7,
									GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11,
									GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};

	for(int i = 0; i < 16; i++) {

		HAL_GPIO_WritePin(GPIOPort, GPIOPins[i], GPIO_PIN_SET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(GPIOPort, GPIOPins[i], GPIO_PIN_RESET);

	}
	for(int i = 15; i >= 0; i--) {

		HAL_GPIO_WritePin(GPIOPort, GPIOPins[i], GPIO_PIN_SET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(GPIOPort, GPIOPins[i], GPIO_PIN_RESET);

	}

}

/*
 * General Falling-Edge EXTI Callback function
 *
 * Internal conditional determines which function to call
 * based on which button was pressed.
 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {

	if(GPIO_Pin == jamButtonPin) {
		jamStart = true;
	}

}

/*
 * Used for LED Timing
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {


	if(htim == timerLED) {
		GPIOB->ODR = leds[songIndex];

		if(songIndex == 503) {
			// Start timer to change PWM duty cycle
			HAL_TIM_Base_Start_IT(timerDim);
			dimNum = 1;
		}
		else if(songIndex == 575) {
			HAL_TIM_Base_Start_IT(timerDim);
			dimNum = 2;
			dimIndex = 0;

			__HAL_TIM_SET_COMPARE(timerPWM[2], PWMChannels[2], systemPWM);
			__HAL_TIM_SET_COMPARE(timerPWM[3], PWMChannels[3], systemPWM);
		}
		else if (songIndex == 629) {
			HAL_TIM_Base_Start_IT(timerDim);
			dimNum = 3;
			dimIndex = 0;

			__HAL_TIM_SET_COMPARE(timerPWM[0], PWMChannels[0], systemPWM);
			__HAL_TIM_SET_COMPARE(timerPWM[1], PWMChannels[1], systemPWM);
		}
		else if (songIndex == 693) {
			dimNum = 4;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			__HAL_TIM_SET_COMPARE(timerPWM[0], PWMChannels[0], systemPWM);
			__HAL_TIM_SET_COMPARE(timerPWM[1], PWMChannels[1], systemPWM);
			__HAL_TIM_SET_COMPARE(timerPWM[2], PWMChannels[2], systemPWM);
			__HAL_TIM_SET_COMPARE(timerPWM[3], PWMChannels[3], systemPWM);

		}
		else if (songIndex == 973) {
			dimNum = 5;
			dimIndex = 0;
			HAL_TIM_Base_Start_IT(timerDim);


		}
		else if (songIndex == timerChangeIndex[0]) {
			HAL_TIM_Base_Stop_IT(timerDim);
			dimIndex = 6;
			dimIndex = 0;
			timerDim->Instance->CNT = 0;
			timerDim->Instance->PSC = 3-1;
			timerDim->Instance->ARR = 50000;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[0] + 8) {
			dimNum = 7;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[0] + 16) {
			dimNum = 8;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[0] + 24) {
			dimNum = 9;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[0] + 32) {
			dimNum = 10;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[0] + 40) {
			dimNum = 11;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[0] + 48) {
			dimNum = 12;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[0] + 56) {
			dimNum = 13;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[0] + 64) {
			dimNum = 135;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[1]) {
			dimNum = 14;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->PSC = 5-1;
			timerDim->Instance->ARR = 60000;
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[1] + 16) {
			dimNum = 15;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[1] + 32) {
			dimNum = 16;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[1] + 48) {
			dimNum = 17;
			dimIndex = 0;
			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[2]) {
			dimNum = 18;
			dimIndex = 0;

			timerPWM[0]->Instance->ARR = 800;
			timerPWM[2]->Instance->ARR = 800;
			HAL_TIM_PWM_Start(timerPWM[0], PWMChannels[0]);
			HAL_TIM_PWM_Start(timerPWM[1], PWMChannels[1]);
			HAL_TIM_PWM_Start(timerPWM[2], PWMChannels[2]);
			HAL_TIM_PWM_Start(timerPWM[3], PWMChannels[3]);

			HAL_TIM_Base_Stop_IT(timerDim);
			timerDim->Instance->PSC = 5-1;
			timerDim->Instance->ARR = 60000;
			timerDim->Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(timerDim);
		}
		else if (songIndex == timerChangeIndex[2] + 64) {

			HAL_TIM_Base_Stop_IT(timerDim);

			dimNum = 19;
			dimIndex = 0;

			timerPWM[0]->Instance->ARR = 100;
			timerPWM[2]->Instance->ARR = 100;
			__HAL_TIM_SET_COMPARE(timerPWM[0], PWMChannels[0], systemPWM);
			__HAL_TIM_SET_COMPARE(timerPWM[1], PWMChannels[1], systemPWM);
			__HAL_TIM_SET_COMPARE(timerPWM[2], PWMChannels[2], systemPWM);
			__HAL_TIM_SET_COMPARE(timerPWM[3], PWMChannels[3], systemPWM);
			HAL_TIM_PWM_Start(timerPWM[0], PWMChannels[0]);
			HAL_TIM_PWM_Start(timerPWM[1], PWMChannels[1]);
			HAL_TIM_PWM_Start(timerPWM[2], PWMChannels[2]);
			HAL_TIM_PWM_Start(timerPWM[3], PWMChannels[3]);

		}

		songIndex++;
	}

	else if(htim == timerDim) {

		if(dimNum == 1) {
			__HAL_TIM_SET_COMPARE(timerPWM[0], PWMChannels[0], dimBuff[dimIndex]);
			__HAL_TIM_SET_COMPARE(timerPWM[1], PWMChannels[1], dimBuff[dimIndex]);
			__HAL_TIM_SET_COMPARE(timerPWM[2], PWMChannels[2], dimBuff[DIM_SIZE - dimIndex - 1]);
			__HAL_TIM_SET_COMPARE(timerPWM[3], PWMChannels[3], dimBuff[DIM_SIZE - dimIndex - 1]);
			dimIndex++;
			if(dimIndex == DIM_SIZE) {
				dimIndex = 0;
			}

		}
		else if (dimNum == 2) {
			__HAL_TIM_SET_COMPARE(timerPWM[0], PWMChannels[0], dimBuff[dimIndex]);
			__HAL_TIM_SET_COMPARE(timerPWM[1], PWMChannels[1], dimBuff[dimIndex]);

			dimIndex++;
			if(dimIndex == DIM_SIZE) {
				dimIndex = 0;
			}
		}
		else if (dimNum == 3) {
			__HAL_TIM_SET_COMPARE(timerPWM[2], PWMChannels[2], dimBuff[dimIndex]);
			__HAL_TIM_SET_COMPARE(timerPWM[3], PWMChannels[3], dimBuff[dimIndex]);

			dimIndex++;
			if(dimIndex == DIM_SIZE) {
				dimIndex = 0;
			}
		}
		else if (dimNum == 5) {
			__HAL_TIM_SET_COMPARE(timerPWM[0], PWMChannels[0], dimBuff[dimIndex]);
			__HAL_TIM_SET_COMPARE(timerPWM[1], PWMChannels[1], dimBuff[dimIndex]);
			__HAL_TIM_SET_COMPARE(timerPWM[2], PWMChannels[2], dimBuff[DIM_SIZE - dimIndex - 1]);
			__HAL_TIM_SET_COMPARE(timerPWM[3], PWMChannels[3], dimBuff[DIM_SIZE - dimIndex - 1]);
			dimIndex++;
			if(dimIndex == DIM_SIZE) {
				dimIndex = 0;
			}
		}
		else if (dimNum >= 6 && dimNum <= 13) {

			if(dimIndex < DIM_SIZE + 100) {
				__HAL_TIM_SET_COMPARE(timerPWM[0], PWMChannels[0], dimBuff2[dimIndex]);
				__HAL_TIM_SET_COMPARE(timerPWM[1], PWMChannels[1], dimBuff2[dimIndex]);
				__HAL_TIM_SET_COMPARE(timerPWM[2], PWMChannels[2], dimBuff2[dimIndex]);
				__HAL_TIM_SET_COMPARE(timerPWM[3], PWMChannels[3], dimBuff2[dimIndex]);
				dimIndex += 1;
			}
		}
		else if ((dimNum >= 13 && dimNum <= 17) || dimNum == 135) {

			if(dimIndex < DIM_SIZE + 100) {
				__HAL_TIM_SET_COMPARE(timerPWM[0], PWMChannels[0], dimBuff2[dimIndex]);
				__HAL_TIM_SET_COMPARE(timerPWM[1], PWMChannels[1], dimBuff2[dimIndex]);
				__HAL_TIM_SET_COMPARE(timerPWM[2], PWMChannels[2], dimBuff2[dimIndex]);
				__HAL_TIM_SET_COMPARE(timerPWM[3], PWMChannels[3], dimBuff2[dimIndex]);
				dimIndex += 1;
			}

		}
		else if (dimNum == 18) {
			if(dimIndex < 801) {
				__HAL_TIM_SET_COMPARE(timerPWM[0], PWMChannels[0], dimBuffLog[dimIndex]);
				__HAL_TIM_SET_COMPARE(timerPWM[1], PWMChannels[1], dimBuffLog[dimIndex]);
				__HAL_TIM_SET_COMPARE(timerPWM[2], PWMChannels[2], dimBuffLog[dimIndex]);
				__HAL_TIM_SET_COMPARE(timerPWM[3], PWMChannels[3], dimBuffLog[dimIndex]);
				dimIndex += 1;
			}
		}

	}


}

/*
 * Displays a non-critical fault to indicate reduced functionality
 */
void dispFault(void) {
	GPIOB->ODR |= 0xFFFF;
}

/*
 * Displays a critical failure and ceases all operations
 */
void dispFailure(void) {

//	HAL_TIM_Base_Stop(timerDelay);
//	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 500 ms)
//	uint32_t timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to
//	bool displayBlink = false;
//
//
//	do {
//
//		if(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal >= (65535 / 4)) {		// Use hardware timer to blink/beep display
//
//			HAL_GPIO_TogglePin(debugLEDPort, debugLEDPin);
//
//			timerVal = __HAL_TIM_GET_COUNTER(timerDelay);				// Update timer value
//
//			displayBlink = !displayBlink;							// Toggle display blink counter
//
//
//
//		}
//
//
//	} while(1);

	GPIOB->ODR |= 0xFFFF;

}

/*
 * Begins DMA streams to pull data from memory, process data, and push to i2s amplifier.
 */
void startAudioStream(void) {

	// We are in jam now.
	inJam = true;

	// Reset so as to not trigger it again
	jamStart = false;

	// Pre-fill TX buffer
	fillTxBuffer(0);
	fillTxBuffer(BUFFER_SIZE);

	// Start TX DMA stream
	HAL_I2S_Transmit_DMA(&hi2s1, i2sTxBuff, BUFFER_SIZE * 2);

	// Enable Amplifier
	if(audioEn) {
		NAU8315YG_AmpEnable(&i2sAmp);
	}

	// Start LED timer
	HAL_TIM_Base_Start_IT(timerLED);

	// Interrupts will take care of the rest.

}

/*
 * Halts DMA streams
 */
void stopAudioStream(void) {

	// Disable Amplifier
	NAU8315YG_AmpDisable(&i2sAmp);

	// Stop DMA Stream
	HAL_I2S_DMAStop(&hi2s1);

	// Reset memory address pointer
	flashReadAddr = initialMemoryOffset;

	// Not jamming anymore
	inJam = false;

	// Reset song index
	songIndex = 0;

	// Stop LED timer
	HAL_TIM_Base_Stop_IT(timerLED);

}

/*
 * DMA completion callbacks
 */

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {

	// Fill first half of i2s TX buffer
	fillTxBuffer(0);


}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {

	// Fill second half of i2s transmit buffer
	fillTxBuffer(BUFFER_SIZE);


}

void fillTxBuffer(uint16_t offset) {

	// Read next chunk of audio data, increment flash read address
	W25Q_readData(&spiFlash, flashReadAddr, BUFFER_SIZE, spiRxBuff);
	flashReadAddr += BUFFER_SIZE;

	// Playing all of a mono file canS-mono-reduced
	 for(uint16_t i = 0; i < BUFFER_SIZE; i += 2) {

		 i2sTxBuff[offset + (i) + 1] = (spiRxBuff[i + 1] << 8) | spiRxBuff[i];

	 }


	// If we have reached the end of the audio clip, reset flash read address
	if(flashReadAddr > audioAddr_END) {
		stopAudioStream();
	}


}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {

	//TEST?
	__NOP();


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
