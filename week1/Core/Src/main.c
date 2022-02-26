/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //1//
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //LED LD2 (PA5)

	  //2//
//	  GPIO_PinState B1State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); // read B1 (PC13)
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, B1State); //LED LD2 (PA5)

	  //3//
//	  static GPIO_PinState B1State[2] = {0};
//	  B1State[0]= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); // read B1 (PC13)
//	  if(B1State[1] == GPIO_PIN_SET && B1State[0] == GPIO_PIN_RESET)
//	  {
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }
//	  B1State[1] = B1State[0];

	  //4//
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  HAL_Delay(500);

	  //5//
//	  static uint32_t timeStamp = 0;
//	  if( HAL_GetTick() - timeStamp >= 500)
//	  {
//		  timeStamp = HAL_GetTick();
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }

	  //6//
	  // button
//	  static GPIO_PinState B1State[2] = {0};
//	  static uint32_t TimeDelay = 500; // ms
//  	  B1State[0]= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); // read B1 (PC13)
//  	  if(B1State[1] == GPIO_PIN_SET && B1State[0] == GPIO_PIN_RESET) //falling edge
//  	  {
//  		  if(TimeDelay == 500)
//  		  {
//  			  TimeDelay = 1000;
//  		  }
//  		  else
//  		  {
//  			  TimeDelay = 500;
//  		  }
//
//  	  }
//  	  B1State[1] = B1State[0];
//
//  	  //LED
//  	  static uint32_t timeStamp = 0;
//	  if( HAL_GetTick() - timeStamp >= TimeDelay)
//	  {
//		  timeStamp = HAL_GetTick();
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }

	  // Exercise //
	  // 1 //
//	  static GPIO_PinState S1[2] = {0};
//	  static uint32_t TimeDelay[4] = {250, 500, 1000, 1500};
//	  static int c = 3;
//	  S1[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); // switch S1 (PA10)
//	  if(S1[0] == GPIO_PIN_RESET && S1[1] == GPIO_PIN_SET) // falling edge
//	  {
//		  c+=1;
//	  }
//	  S1[1] = S1[0];
//
//	  static uint32_t timeStamp = 0;
//
//	  if( HAL_GetTick() - timeStamp >= TimeDelay[c%4])
//  	  {
//  		  timeStamp = HAL_GetTick();
//  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9); // LED D1 (PA9)
//  	  }

	  // 2 //

//	  static GPIO_PinState S1[2] = {0};
//	  static uint32_t TimeDelay[2] = {0, 1000};
//	  static int c = 1;
//	  S1[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); // switch S1 (PA10)
//	  if(S1[0] == GPIO_PIN_RESET && S1[1] == GPIO_PIN_SET) // falling edge
//	  {
//		  c+=1;
//	  }
//	  S1[1] = S1[0];
//
//	  static uint32_t timeStamp = 0;
//
//	  if( HAL_GetTick() - timeStamp >= TimeDelay[c%2])
//		  {
//			  timeStamp = HAL_GetTick();
//			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9); // LED D1 (PA9)
//		  }

	  // 3 //

	  static GPIO_PinState S1[2] = {0};
	  static uint32_t TimeDelay[2] = {500, 1500};
	  static int c = 1;
	  S1[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); // switch S1 (PA10)
	  if(S1[0] == GPIO_PIN_RESET && S1[1] == GPIO_PIN_SET) // falling edge
	  {
		  c+=1;
	  }
	  S1[1] = S1[0];

	  static uint32_t timeStamp = 0;

	  if( HAL_GetTick() - timeStamp <= TimeDelay[c%2])
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); // LED D1 (PA9)
	  }
	  else if( HAL_GetTick() - timeStamp >= TimeDelay[c%2] && HAL_GetTick() - timeStamp <= 2000)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // LED D1 (PA9)
	  }
	  else
	  {
		  timeStamp = HAL_GetTick();
	  }





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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin D1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : S1_Pin */
  GPIO_InitStruct.Pin = S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(S1_GPIO_Port, &GPIO_InitStruct);

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

