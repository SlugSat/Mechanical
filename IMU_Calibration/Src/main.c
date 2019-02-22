/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** 
	* CORRECTION EQUATION: x = (scaling factor)*(x_meas + offset)
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <BNO055_IMU.h>
#include <string.h>
#include <float.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SKIP_GYRO

#define GYRO_POINTS 1000
#define GYRO_WAIT_TIME_MS 10

#define MAG_POINTS 500
#define MAG_WAIT_TIME_MS 50

#define GYRO_SPHERE_R 1.0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

double max(double x, double y) {
	if(x > y) {
		return x;
	}
	return y;
}

double min(double x, double y) {
	if(x < y) {
		return x;
	}
	return y;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	do {
		char transmit[50];
		sprintf(transmit, "Initializing IMU...\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
	}
	while(0);
	
	IMU_init(&hi2c1, OPERATION_MODE_MAGGYRO);
	
	
	// GYRO CALIBRATION
	#ifndef SKIP_GYRO
	do {
		char transmit[10];
		sprintf(transmit, "GYRO\r\nWaiting for gyro to warm up...\r\n\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
	}
	while(0);
	
	HAL_Delay(10000);
	
	do {
		char transmit[200];
		sprintf(transmit, "Leave the IMU flat on the table and press the button\r\n...\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
	}
	while(0);
	
	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET); // Block until button is pressed
	
	do {
		char transmit[200];
		sprintf(transmit, "Gyro calibrating... ");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
	}
	while(0);
	
	float gyro_data[3];
	
	double gyro_x_offset = 0;
	double gyro_y_offset = 0;
	double gyro_z_offset = 0;
	
	for(int i = 0;i < GYRO_POINTS;i++) {
		get_gyr_data(&hi2c1, gyro_data);
		gyro_x_offset += gyro_data[0]/GYRO_POINTS;
		gyro_y_offset += gyro_data[1]/GYRO_POINTS;
		gyro_z_offset += gyro_data[2]/GYRO_POINTS;
		
		HAL_Delay(GYRO_WAIT_TIME_MS);
	}
	
	do {
		char transmit[200];
		sprintf(transmit, "DONE\r\n\r\n\tGyro offsets\r\nx: %5.6f\ty: %5.6f\tz: %5.6f\r\n\r\n", gyro_x_offset, gyro_y_offset, gyro_z_offset);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 40);
	}
	while(0);
	#endif
	
	
	// MAG CALIBRATION
	do {
		char transmit[10];
		sprintf(transmit, "MAGNETOMETER\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
	}
	while(0);
	
	HAL_Delay(500);
	
	do {
		char transmit[200];
		sprintf(transmit, "Press the button then move the IMU slowly in a spherical motion. Once you are done taking readings press the button again.\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
	}
	while(0);
	
	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET); // Block until button is pressed
	
	do {
		char transmit[200];
		sprintf(transmit, "Reading magnetometer... ");
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 20);
	}
	while(0);
	
	double mag_x_max = -DBL_MAX;
	double mag_y_max = -DBL_MAX;
	double mag_z_max = -DBL_MAX;
	double mag_x_min = DBL_MAX;
	double mag_y_min = DBL_MAX;
	double mag_z_min = DBL_MAX;
	
	float mag_data[3];
	
	// Find maximum/minimum mag values
	//for(int i = 0;i < MAG_POINTS;i++) {
	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) { // Block until button is pressed
		get_mag_data(&hi2c1, mag_data);
		mag_x_max = max(mag_data[0], mag_x_max);
		mag_y_max = max(mag_data[1], mag_y_max);
		mag_z_max = max(mag_data[2], mag_z_max);
		
		mag_x_min = min(mag_data[0], mag_x_min);
		mag_y_min = min(mag_data[1], mag_y_min);
		mag_z_min = min(mag_data[2], mag_z_min);
		
		HAL_Delay(MAG_WAIT_TIME_MS);
	}
	
	// Send max/min values
	do {
		char transmit[200];
		sprintf(transmit, "x: \ty: \tz: \r\nmax: %5.6f\t %5.6f\t %5.6f\r\nmin: %5.6f\t %5.6f\t %5.6f\r\n",
			mag_x_max, mag_y_max, mag_z_max, mag_x_min, mag_y_min, mag_z_min);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 40);
	}
	while(0);
	
	// Calculate scaling factor and offsets
	double mag_x_scalar = 2*GYRO_SPHERE_R/(mag_x_max - mag_x_min);
	double mag_y_scalar = 2*GYRO_SPHERE_R/(mag_y_max - mag_y_min);
	double mag_z_scalar = 2*GYRO_SPHERE_R/(mag_z_max - mag_z_min);
	
	double mag_x_offset = -(mag_x_max + mag_x_min)/2;
	double mag_y_offset = -(mag_y_max + mag_y_min)/2;
	double mag_z_offset = -(mag_z_max + mag_z_min)/2;
	
	do {
		char transmit[200];
		sprintf(transmit, "\tMag offsets\r\nx: %5.6f\ty: %5.6f\tz: %5.6f\r\n\r\n\tMag scaling factors\r\nx: %5.6f\ty: %5.6f\tz: %5.6f\r\n",
			mag_x_offset, mag_y_offset, mag_z_offset, mag_x_scalar, mag_y_scalar, mag_z_scalar);
		HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 40);
	}
	while(0);
	
  /* USER CODE END 2 */

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
