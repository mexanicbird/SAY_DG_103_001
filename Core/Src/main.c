/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdio.h"
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
bool flag1; // ����� ��� ��������
bool flag2; // ����� ��� �������
unsigned long t1, t2, t3; // ��� ������� �������
uint8_t tx_buffer1[] = "SAY_Diesel engine\n\r"; // ������� ����� � �������
uint8_t tx_buffer2[] = "Varable:\n\r"; // ������� ����� � �������
uint8_t tx_buffer3[] = "\n\r"; // ������� ����� � �������
uint8_t msg1[64];  // ������ ��� �����
uint8_t msg2[64];  // ������ ��� �����
uint8_t msg3[64];  // ������ ��� �����
uint8_t msg4[64];  // ������ ��� �����
uint8_t msg5[64];  // ������ ��� �����
float x2 = 33.12;// �����
uint32_t adc_val[4];
int adc_val1;
int adc_val2;
int adc_val3;
int adc_val4;
int rx_buffer_int = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//HAL_UART_Transmit_DMA(&huart1, rx_buffer, 10); // отправка обратно для проверки
	//HAL_UART_Receive_IT(&huart1, rx_buffer, 4);
//}
void Huart(){
	 if(HAL_GetTick() - t2 >=1000) {
		 t2 = HAL_GetTick();
	     HAL_UART_Transmit(&huart1, tx_buffer1, sizeof tx_buffer1/sizeof tx_buffer1[0], 0xFFFF);
	     HAL_UART_Transmit(&huart1, tx_buffer2, sizeof tx_buffer2/sizeof tx_buffer2[0], 0xFFFF);
	     HAL_UART_Transmit(&huart1, msg1, sprintf((char *)msg1, "Rotation speed reference = %d\n\r", adc_val1), 0xFFFF);
	     HAL_UART_Transmit(&huart1, msg2, sprintf((char *)msg2, "PID_Kp = %d", adc_val2), 0xFFFF);
	     HAL_UART_Transmit(&huart1, msg4, sprintf((char *)msg4, "   PID_Kd = %d", adc_val3), 0xFFFF);
	     HAL_UART_Transmit(&huart1, msg5, sprintf((char *)msg5, "   PID_Ki = %d\n\r", adc_val4), 0xFFFF);
	     //HAL_UART_Transmit(&huart1, msg3, sprintf((char *)msg3, " Varable_3 = %.2f\n\r", x2), 0xFFFF);
	     HAL_UART_Transmit(&huart1, tx_buffer3, sizeof tx_buffer3/sizeof tx_buffer3[0], 0xFFFF);
	 }
}

void Blink(){
	 if (flag1 == 1) {
		 if (HAL_GetTick() - t1 >= 2000) {
			 flag1 = 0;
			 t1 = HAL_GetTick();
		 }
	 }
	 if (flag1 == 0) {
	 		 if (HAL_GetTick() - t1 >= 300) {
	 		flag1 = 1;
	 		t1 = HAL_GetTick();
	 		 }
	 	HAL_GPIO_WritePin(GPIOC, DO1_Pin, flag1);
	 	 }

 }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char str[100];
	uint8_t rx_buffer[5]={0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	t1 = HAL_GetTick(); // ��� ��������
	t2 = HAL_GetTick(); // ��� �����
	t3 = HAL_GetTick(); // ��� �������
	flag1 = 1;
	flag2 = 1;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, rx_buffer, 4);
  HAL_ADCEx_Calibration_Start(&hadc1);  // ���������� ��� ��� ���������

  LCD_ini();
  sprintf(str,"DESIGN");
  LCD_SetPos(6, 0);
  LCD_String(str);
  HAL_Delay(700);

  sprintf(str,"BY");
  LCD_SetPos(8, 1);
  LCD_String(str);
  HAL_Delay(700);

  sprintf(str,"MEXANICBIRD");
  LCD_SetPos(4, 2);
  LCD_String(str);
  HAL_Delay(3000);
  LCD_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // опрашиваем резисторы //
	  HAL_ADC_Start_DMA(&hadc1, adc_val, 4); // запуск преобразования сигнала
	  adc_val1 = adc_val[0] / 1.46;
	  adc_val2 = adc_val[1] / 40;
	  adc_val3 = adc_val[2] / 135;
	  adc_val4 = adc_val[3] / 135;

	  //if(huart1.RxXferCount==0){
		  rx_buffer[4]=0;
		  HAL_UART_Receive_IT(&huart1, rx_buffer, 4);
	 // }

	  //rx_buffer_int = rx_buffer[0]*1000 + rx_buffer[1]*100 + rx_buffer[2]*10 + rx_buffer[3];

	  Blink(); // вызываем моргалку
	  //Huart(); // вызываем передачу в порт


	  sprintf(str,"System VAL&SET ");
	  LCD_SetPos(3, 0);
	  LCD_String(str);

	  sprintf(str,"Z = %d", adc_val1);
	  LCD_SetPos(0, 1);
	  LCD_String(str);

	  sprintf(str,"S = %s", rx_buffer);
	  LCD_SetPos(11, 1);
	  LCD_String(str);

	  sprintf(str,"Kp = %d", adc_val2);
	  LCD_SetPos(0, 2);
	  LCD_String(str);

	  sprintf(str,"Ki = %d", adc_val3);
	  LCD_SetPos(11, 2);
	  LCD_String(str);

	  sprintf(str,"Kd = %d", adc_val4);
	  LCD_SetPos(0, 3);
	  LCD_String(str);

	  sprintf(str,"D = 100");
	  LCD_SetPos(11, 3);
	  LCD_String(str);


	  if (flag2 == 1) {
		if (HAL_GetTick() - t3 >= 1300) {
			flag2 = 0;
			t3 = HAL_GetTick();
		}
	}
	  if (flag2 == 0) {
		 if (HAL_GetTick() - t3 >= 1) {
		   flag2 = 1;
		   t3 = HAL_GetTick();
		 	}
		  LCD_Clear();

	}	  
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DO1_Pin */
  GPIO_InitStruct.Pin = DO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO1_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
