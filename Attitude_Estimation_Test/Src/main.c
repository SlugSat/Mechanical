/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** Attitude estimation test program. Includes unit tests on the Matrix and
	* AttitudeEstimation functions as well as a main loop which runs attitude
	* estimation with the same parameters as the example Matlab program.
	*
	* Working as of 5/8/2019, but the feedback constants are no longer the same as
	* Matlab.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <AttitudeEstimation.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define ATTITUDE_ESTIMATION_UNIT_TEST

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	char transmit[200];
	
	#ifdef ATTITUDE_ESTIMATION_UNIT_TEST // Begin unit testing code
	
	// ***** MATRIX UNIT TESTS *****
	sprintf(transmit, "--------------------\r\n");	
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	// Test matrixEquals()
	Matrix m = initializeDCM(0, 0, 0);
	
	if(matrixEquals(m, m)) {
		sprintf(transmit, "matrixEquals() PASSED\r\n");
	}
	else {
		sprintf(transmit, "matrixEquals() FAILED\r\n");	
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	sprintf(transmit, "--------------------\r\n");	
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	// Test matrixCopyArray()
	sprintf(transmit, "Testing matrixCopyArray()\r\n");	
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	Matrix m1 = newMatrix(3, 3);
	Matrix m2 = newMatrix(3, 3);
	{
		float row1[] = {1, 2, 3};
		float row2[] = {4, 5, 6};
		float row3[] = {7, 8, 9};
		float* array[] = {row1, row2, row3};
		matrixCopyArray(m1, array);
		printMatrix(m1, transmit);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	}
	
	sprintf(transmit, "--------------------\r\n");	
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	// Test matrixAdd() in place
	{
		matrixAdd(m1, m1, m1);
		float row1[] = {2, 4, 6};
		float row2[] = {8, 10, 12};
		float row3[] = {14, 16, 18};
		float* array[] = {row1, row2, row3};
		matrixCopyArray(m2, array);
		printMatrix(m1, transmit);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
		if(matrixEquals(m1, m2)) {
			sprintf(transmit, "matrixAdd() in place PASSED\r\n");
		}
		else {
			sprintf(transmit, "matrixAdd() in place FAILED\r\n");
		}
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	}
	
	sprintf(transmit, "--------------------\r\n");	
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	// Test matrixMult()
	{
		matrixMult(m2, m2, m1);
		printMatrix(m1, transmit);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
		float row1[] = {120, 144, 168};
		float row2[] = {264, 324, 384};
		float row3[] = {408, 504, 600};
		float* array[] = {row1, row2, row3};
		matrixCopyArray(m2, array);
		if(matrixEquals(m1, m2)) {
			sprintf(transmit, "matrixMult() PASSED\r\n");
		}
		else {
			sprintf(transmit, "matrixMult() FAILED\r\n");
		}
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	}
	
	sprintf(transmit, "--------------------\r\n");	
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	// Test matrixTranspose()
	{
		float row1[] = {1, 2, 3};
		float row2[] = {4, 5, 6};
		float row3[] = {7, 8, 9};
		float* array[] = {row1, row2, row3};
		matrixCopyArray(m1, array);
		matrixTranspose(m1, m2);
		printMatrix(m2, transmit);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	}
	{
		float row1[] = {1, 4, 7};
		float row2[] = {2, 5, 8};
		float row3[] = {3, 6, 9};
		float* array[] = {row1, row2, row3};
		matrixCopyArray(m1, array);
		if(matrixEquals(m1, m2)) {
			sprintf(transmit, "matrixTranspose() PASSED\r\n");
		}
		else {
			sprintf(transmit, "matrixTranspose() FAILED\r\n");
		}
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	}
	
	sprintf(transmit, "--------------------\r\n");	
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	// Test matrixScale()
	matrixScale(m1, 2);
	printMatrix(m1, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	{
		float row1[] = {2, 8, 14};
		float row2[] = {4, 10, 16};
		float row3[] = {6, 12, 18};
		float* array[] = {row1, row2, row3};
		matrixCopyArray(m2, array);
		if(matrixEquals(m1, m2)) {
			sprintf(transmit, "matrixScale() PASSED\r\n");
		}
		else {
			sprintf(transmit, "matrixScale() FAILED\r\n");
		}
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	}
	
	sprintf(transmit, "--------------------\r\n");	
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	// Test vectorRcross()
	Matrix v = make3x1Vector(1, 3, 5);
	{
		vectorRcross(v, m1);
		printMatrix(m1, transmit);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	}
	{
		float row1[] = {0, -5, 3};
		float row2[] = {5, 0, -1};
		float row3[] = {-3, 1, 0};
		float* array[] = {row1, row2, row3};
		matrixCopyArray(m2, array);
		if(matrixEquals(m1, m2)) {
			sprintf(transmit, "vectorRcross() PASSED\r\n");
		}
		else {
			sprintf(transmit, "vectorRcross() FAILED\r\n");
		}
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	}
	
	sprintf(transmit, "--------------------\r\n");	
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	// Test vectorNorm()
	float vnorm = vectorNorm(v);
	sprintf(transmit, "%f\r\n", vnorm);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	if(vnorm != 5.9160797831f) {
		sprintf(transmit, "vectorNorm() FAILED\r\n");
	}
	else {
		sprintf(transmit, "vectorNorm() PASSED\r\n");
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	sprintf(transmit, "--------------------\r\n");	
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	// Test vectorDotProduct()
	float vdotv = vectorDotProduct(v, v);
	sprintf(transmit, "%f\r\n", vdotv);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	if(vdotv != 35) {
		sprintf(transmit, "vectorDotProduct() FAILED\r\n");
	}
	else {
		sprintf(transmit, "vectorDotProduct() PASSED\r\n");
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
	
	while(1);
	
	#endif // End unit testing code
	
	
	
	
	
	
	// ***** INITIALIZE ATTITUDE CONTROL TEST *****
	
	ACSType acs;
	initializeACS(&acs);
	
	acs.gyro_vector = make3x1Vector(0.1, 0.1, 0.1);
	acs.sv_inertial = make3x1Vector(1, 0, 0);
	acs.mag_inertial = make3x1Vector(0, 0, -1);
	
	acs.dt = 1;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		updateAttitudeEstimate(&acs);
		
		// Print DCM every step
		printMatrix(acs.R, transmit);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
		
		sprintf(transmit, "\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
		
		printMatrix(acs.gyro_bias, transmit);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
		
		sprintf(transmit, "\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
		
		HAL_Delay(500);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
