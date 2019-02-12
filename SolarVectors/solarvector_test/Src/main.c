/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// #define YAWPITCH_TEST // Uncomment to unit test yaw and pitch function
// #define USE_ADC_THRESHOLDS // Uncomment to ignore readings on the "dark side" of the satellite

#define ADC_CHANNEL_XP 0
#define ADC_CHANNEL_XN 1
#define ADC_CHANNEL_YP 2
#define ADC_CHANNEL_YN 3
#define ADC_CHANNEL_ZP 4
#define ADC_CHANNEL_ZN 5
#define NUM_ADC_CHANNELS 6

#define R (double)1000.0 // In ohms

#define ADC_MAX_VOLTS 3.3
#define ADC_RESOLUTION 12
#define ADC_TO_VOLTS(adc_raw) ((double)(((adc_raw)*3.3)/(1<<ADC_RESOLUTION)))

#define PI 3.1416

#define ADC_READING_LOW_THRESHOLD 128 // Minimum value to treat the reading as nonzero
#define ADC_READING_HIGH_THRESHOLD 2048 // Minimum value to treat the reading as full exposure (close to 90deg incident angle)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

ADC_ChannelConfTypeDef sConfig = {0};
uint32_t adc_data[NUM_ADC_CHANNELS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/** 
 * @brief  Blocking function to read from ADC1
 * @param  A predefined ADC channel
 * @return 12-bit conversion result
*/

/** 
 * @brief  Prints yaw and pitch of a 3-vector over UART
 * @param  Vector components
 * @return None
*/
void PrintYawPitch(double vector_x, double vector_y, double vector_z) {
	// Convert vector into yaw/pitch angles referenced to the +X unit vector (1, 0, 0)
	double yaw_degrees = 180*acos(vector_x/sqrt(pow(vector_x, 2) + pow(vector_y, 2)))/PI;
	if(vector_y < 0) {
		yaw_degrees *= -1;
	}
	
	double pitch_degees = 180*acos(sqrt(pow(vector_x, 2) + pow(vector_y, 2)))/PI;
	if(vector_z < 0) {
		pitch_degees *= -1;
	}
	
	// Print results over UART
	char transmit[50];
	sprintf(transmit, "yaw: %3.1f pitch: %3.1f\r\n", yaw_degrees, pitch_degees);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 40);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	#ifdef YAWPITCH_TEST // Unit testing
	int num_tests = 7;
	double x_components[] = {1, 0.7071, -0.7071, 0.7071, -0.7071, 0.7071, 0};
	double y_components[] = {0, 0.7071, 0.7071, -0.7071, -0.7071, 0, -0.7071};
	double z_components[] = {0, 0, 0, 0, 0, 0.7071, -0.7071};
	double desired_yaw[] = {0, 45, 135, -45, -135, 0, -90};
	double desired_pitch[] = {0, 0, 0, 0, 0, 45, -45};
	
	for(int i = 0;i < num_tests;i++) {
		PrintYawPitch(x_components[i], y_components[i], z_components[i]);
		char transmit[50];
		sprintf(transmit, "Desired:\r\nyaw: %3.1f pitch: %3.1f\r\n\r\n", desired_yaw[i], desired_pitch[i]);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 50);
	}
	
	while(1);
	#endif
	
	do {
		char transmit[50];
		sprintf(transmit, "Before ADC Start\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
	}
	while(0);
	
	HAL_ADC_Start_DMA(&hadc1, adc_data, NUM_ADC_CHANNELS); // Start ADC in DMA mode
	
	do {
		char transmit[50];
		sprintf(transmit, "Finished init\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
	}
	while(0);
	
	// See the Attitude Control System document for info on the body reference frame and axes
	double volts_xp, volts_xn, volts_yp, volts_yn, volts_zp, volts_zn; // Positive and negative raw current values for each axis
	
	double vector_mag, vector_x, vector_y, vector_z; // Components of the solar vector in the body frame

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Wait for button press
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_SET);

		// Read data and convert to volts
		volts_xp = ADC_TO_VOLTS(adc_data[ADC_CHANNEL_XP]);
		volts_xn = ADC_TO_VOLTS(adc_data[ADC_CHANNEL_XN]);
		volts_yp = ADC_TO_VOLTS(adc_data[ADC_CHANNEL_YP]);
		volts_yn = ADC_TO_VOLTS(adc_data[ADC_CHANNEL_YN]);
		volts_zp = ADC_TO_VOLTS(adc_data[ADC_CHANNEL_ZP]);
		volts_zn = ADC_TO_VOLTS(adc_data[ADC_CHANNEL_ZN]);
		
		do {
			char transmit[50];
			sprintf(transmit, "zp: %.2f zn: %.2f\r\n", volts_zp, volts_zn);
			HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 40);
		}
		while(0);
		
		// Calculate magnitude of solar vector
		vector_mag = sqrt(pow(volts_xp - volts_xn, 2) + pow(volts_yp - volts_yn, 2) + pow(volts_zp - volts_zn, 2));
		
		// Find normalized solar vector components
		vector_x = (volts_xp - volts_xn)/vector_mag;
		vector_y = (volts_yp - volts_yn)/vector_mag;
		vector_z = (volts_zp - volts_zn)/vector_mag;
		
		PrintYawPitch(vector_x, vector_y, vector_z);
		
		// Wait for button release
    while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_RESET);
		
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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
  /**Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
