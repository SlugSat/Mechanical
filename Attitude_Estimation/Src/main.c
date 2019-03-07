/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This program runs attitude estimation on our test hardware using solar
	* vectors and IMU readings.
	*
	* See the Attitude Estimation SOP.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <AttitudeEstimation.h>
#include <SolarVectors.h>
#include <BNO055_IMU.h>
#include <string.h>
#include <DigitalFilters.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define VERBOSE_PRINTING

#define NUM_SOLAR_PANELS 6
#define MAG_HIST_LENGTH 10
#define SV_HIST_LENGTH 10

#define KP_MAG_BASE 0 //1.0
#define KI_MAG_BASE 0 //0.3
#define KP_SV_BASE 0 //1.0
#define KI_SV_BASE 0 //0.3

#define SET_DT 50 // In ms

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t sv_raw[NUM_SOLAR_PANELS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	char transmit[200];
	
	// ***** INITIALIZE SENSORS *****
	IMU_init(&hi2c1, OPERATION_MODE_MAGGYRO);
	float gyro_data[3] = {0};
	float mag_data[3] = {0};
	
	HAL_ADC_Start_DMA(&hadc1, sv_raw, NUM_SOLAR_PANELS); // Start ADC in DMA mode
	float sv_data[NUM_SOLAR_PANELS] = {0};
	
	// ***** INITIALIZE MOVING AVERAGE FILTERS *****
	MovingAvgFilter mag_filter = newMovingAvgFilter(3, MAG_HIST_LENGTH);
	MovingAvgFilter sv_filter = newMovingAvgFilter(NUM_SOLAR_PANELS, SV_HIST_LENGTH);
	
	// Run filters to fill them with initial data
	for(int i = 0;i < MAG_HIST_LENGTH && i < SV_HIST_LENGTH;i++) {
		// Read magnetometer
		get_mag_data(&hi2c1, mag_data);
		runMovingAvgFilter(mag_filter, mag_data);
		
		// Read solar vector
		for(int i = 0;i < NUM_SOLAR_PANELS;i++) {
			sv_data[i] = ADC_TO_VOLTS(sv_raw[i]);
		}
		runMovingAvgFilter(sv_filter, sv_data);
		
		HAL_Delay(SET_DT);
	}
	
	// ***** INITIALIZE MATRICES *****
	Matrix gyro_vector = newMatrix(3, 1);
	Matrix mag_vector = newMatrix(3, 1);
	Matrix solar_vector = newMatrix(3, 1);
	Matrix sv_inertial = make3x1Vector(1, 0, 0);
	Matrix mag_inertial = make3x1Vector(0, -1, 0);
	
	// ***** GYRO FEEDBACK *****
	float Kp_mag = KP_MAG_BASE;
	float Ki_mag = KI_MAG_BASE;
	float Kp_sv = KP_SV_BASE;
	float Ki_sv = KI_SV_BASE;
	
	Matrix bias_estimate = make3x1Vector(0, 0, 0);
	
	float dt = SET_DT/1000.0; // In seconds
	Matrix R = initializeDCM(0, 0, 0);
	float y_p_r[3]; // Yaw/pitch/roll
	
	sprintf(transmit, "Finished init\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Read gyro and transform into a column vector Matrix
		get_gyr_data(&hi2c1, gyro_data);
		vectorCopyArray(gyro_vector, gyro_data, 3);
		
		// Read magnetometer, iterate moving average filter, transform into a vector Matrix
		get_mag_data(&hi2c1, mag_data);
		runMovingAvgFilter(mag_filter, mag_data);
		vectorCopyArray(mag_vector, mag_data, 3);
		
		// Read solar vector
		for(int i = 0;i < NUM_SOLAR_PANELS;i++) {
			sv_data[i] = ADC_TO_VOLTS(sv_raw[i]);
		}
		runMovingAvgFilter(sv_filter, sv_data);
		SV_Status sv_return = findSolarVector(sv_data, NUM_SOLAR_PANELS, solar_vector);
		
		if(sv_return == SV_FOUND) {
			Kp_sv = KP_SV_BASE;
			Ki_sv = KI_SV_BASE;
		}
		else {
			Kp_sv = 0;
			Ki_sv = 0;
		}
		
		// DCM integration
		integrateDCM(R, bias_estimate, gyro_vector, mag_vector, solar_vector, mag_inertial, sv_inertial,
								Kp_mag, Ki_mag, Kp_sv, Ki_sv, dt);
		
		#ifdef VERBOSE_PRINTING
		// Print gyro vector
		sprintf(transmit, "Gyro vector:\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
		printMatrix(gyro_vector, transmit);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
		
		// Filtered mag vector
		sprintf(transmit, "Mag vector:\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
		printMatrix(mag_vector, transmit);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
		
		// Print solar vector
		sprintf(transmit, "Solar vector:\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
		printMatrix(solar_vector, transmit);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
		
		// Print new DCM
		sprintf(transmit, "New DCM:\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
		printMatrix(R, transmit);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 200);
		sprintf(transmit, "\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 10);
		#endif
		
		// Print Euler angles
		findEulerAngles(R, y_p_r);
		sprintf(transmit, "Yaw:   %6.2f\r\nPitch: %6.2f\r\nRoll:  %6.2f\r\n\r\n", 180.0*y_p_r[0]/3.1416, 180.0*y_p_r[1]/3.1416, 180.0*y_p_r[2]/3.1416);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 40);
		
		HAL_Delay(SET_DT);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
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
