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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<math.h>
#include<stdlib.h>
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

	uint32_t read_adc_channel(uint32_t channel) {
		ADC_ChannelConfTypeDef sConfig = {0};

		sConfig.Channel = channel;			//Cycles setting different channels to read
		sConfig.Rank = 1;					//Prioritizes current channel
		sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		sConfig.Offset = 0;

		//If channel configuration fails enter error handler
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		return HAL_ADC_GetValue(&hadc1);

	}


	int __io_putchar(int ch)
	{
	    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	    return ch;
	}
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	 unsigned int arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
	 int n, s, e, w;
	 GPIO_PinState inA_ns, inB_ns, inA_ew, inB_ew;
	 int nsDiff, ewDiff;
	 int CCR_ch1, CCR_ch2;
	 int pwmMax = (int)(0.85*arr);
	 int pwmMin = (int)(0.25*arr);
	 int pwmStart = 100;
	 int pwmDead = 75;
	 int maxDiff = 340;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  n = read_adc_channel(ADC_CHANNEL_10);
	  s = read_adc_channel(ADC_CHANNEL_11);
	  w = read_adc_channel(ADC_CHANNEL_12);
	  e = read_adc_channel(ADC_CHANNEL_13);
	  nsDiff = n-s;
	  ewDiff = e-w;
//==============================Linear Actuator(NORTH/SOUTH)===================================
	  //If difference is greater than 100 start
	  if(abs(nsDiff) >= pwmStart) {

		  //If difference is greater than 340 clamp to max PWM speed
		  if(abs(nsDiff) > maxDiff){
			  //Handles direction based on the NORTH/SOUTH difference sign (MAX PWM)
			  if(nsDiff > 0) {
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				  CCR_ch1 = pwmMax;
				  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CCR_ch1);

				  //Reading H-bridge directional pins into variables for debugging
				  inA_ns = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
				  inB_ns = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
			  }
			  //If difference is negative reverse direction of linear actuator at max PWM
			  else {
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
				  CCR_ch1 = pwmMax;
				  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CCR_ch1);

				  //Reading H-bridge directional pins into variables for debugging
				  inA_ns = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
				  inB_ns = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
			  }
		  }
		  //Handles direction based on the NORTH/SOUTH difference sign (Variable PWN)
		  else if(nsDiff > 0) {
			  //Tilt South (set GPIO pin for forward direction INA and clear INB)
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			  CCR_ch1 = pwmMin + ((pwmMax-pwmMin)*abs(nsDiff)) / maxDiff;
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CCR_ch1);

			  //Reading H-bridge directional pins into variables for debugging
			  inA_ns = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
			  inB_ns = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
		  }
		  //If difference is negative reverse direction of linear actuator at variable PWM
		  else {
			  //Tilt North (set GPIO pin for reverse direction INB and clear INA)
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
			  CCR_ch1 = pwmMin + ((pwmMax-pwmMin)*abs(nsDiff)) / maxDiff;
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CCR_ch1);

			  //Reading H-bridge directional pins into variables for debugging
			  inA_ns = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
			  inB_ns = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
		  }
	  }
	  //If difference is less than 70 stop the linear actuator
	  else if(abs(nsDiff) < pwmDead) {
		  CCR_ch1 = 0;
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CCR_ch1);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

		  //Reading H-bridge directional pins into variables for debugging
		  inA_ns = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
		  inB_ns = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	  }
//=====================================================================================
//==============================WORM GEAR(EAST/WEST)===================================
	  //If difference is greater than 100 start
	  if(abs(ewDiff) >= pwmStart) {
		  //If difference is greater than 340 clamp to max PWM speed
		  if(abs(ewDiff) > maxDiff){
			  //Handles direction based on the EAST/WEST difference sign (MAX PWM)
			  if(ewDiff > 0) {
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				  CCR_ch2 = pwmMax;
				  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CCR_ch2);

				  //Reading H-bridge directional pins into variables for debugging
				  inA_ew = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
				  inB_ew = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
			  }
			  //If difference is negative reverse direction of worm gear at max PWM
			  else {
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				  CCR_ch2 = pwmMax;
				  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CCR_ch2);

				  //Reading H-bridge directional pins into variables for debugging
				  inA_ew = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
				  inB_ew = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
			  }
		  }
		  //Handles direction based on the EAST/WEST difference sign (Variable PWM)
		  else if(ewDiff > 0) {
			  //Tilt West (set GPIO pin for forward direction INA and clear INB)
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			  CCR_ch2 = pwmMin + ((pwmMax-pwmMin)*abs(ewDiff)) / maxDiff;
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CCR_ch2);

			  //Reading H-bridge directional pins into variables for debugging
			  inA_ew = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
			  inB_ew = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
		  }
		  //If difference is negative reverse direction of worm gear at variable PWM
		  else {
			  //Tilt East (set GPIO pin for reverse direction INB and clear INA)
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			  CCR_ch2 = pwmMin + ((pwmMax-pwmMin)*abs(ewDiff)) / maxDiff;
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CCR_ch2);

			  //Reading H-bridge directional pins into variables for debugging
			  inA_ew = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
			  inB_ew = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
		  }
	  }
	  //If difference is less than 70 stop the worm gear
	  else if(abs(ewDiff) < pwmDead) {
		  CCR_ch2 = 0;
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CCR_ch2);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

		  //Reading H-bridge directional pins into variables for debugging
		  inA_ew = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		  inB_ew = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	  }
//=====================================================================================
	printf("Linear Actuator Value: %4d|Worm Gear Value: %4d|nsDiff: %4d|ewDiff: %4d|Extend(LA): %4d Retract(LA): %4d| Forward(WG): %4d Reverse(WG): %4d\r\n", CCR_ch1, CCR_ch2, nsDiff, ewDiff, inA_ns, inB_ns, inA_ew, inB_ew);
	HAL_Delay(1000);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 874;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
