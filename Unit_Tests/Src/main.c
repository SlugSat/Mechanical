/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <STM32SerialCommunication.h>
#include <AttitudeEstimation.h>
#include <FeedbackControl.h>
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
	char transmit[200];
	sprintf(transmit, "ACS UNIT TEST PROGRAM\r\nBy Mechanical team\r\n\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	
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
	
	
	// Initialize ACS struct
	sprintf(transmit, "Initializing ACS struct... ");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	ACSType acs;
	initializeACS(&acs);
	initializeACSSerial(&huart2);
	
	sprintf(transmit, "Initialization done!\r\n\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	sprintf(transmit, "J_body\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.J_body, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	sprintf(transmit, "J_body_inv\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.J_body_inv, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	sprintf(transmit, "J_rw\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.J_rw, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	sprintf(transmit, "J_rw_inv\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.J_rw_inv, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	
	// ***** UNIT TESTS *****
	// ----- Test Matrix functions -----
	// vectorCrossProduct()
	Matrix v1 = make3x1Vector(1, 2, 3);
	Matrix v2 = make3x1Vector(4, 5, 6);
	Matrix v1xv2 = make3x1Vector(-3, 6, -3);
	Matrix x = newMatrix(3, 1);
	vectorCrossProduct(v1, v2, x);
	if(matrixEquals(x, v1xv2)) {
		sprintf(transmit, "vectorCrossProduct() PASSED\r\n");
	}
	else {
		sprintf(transmit, "vectorCrossProduct() FAILED\r\n");
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	
	// ----- Test findErrorVectors() -----
	sprintf(transmit, "\r\nTesting findErrorVectors()\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	// TEST 1
	sprintf(transmit, "----- TEST 1 -----\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	acs.Rt = initializeDCM(0, 0, 0);
	vectorSetXYZ(acs.craft_inertial, 1, 2, 3);
	float c_i_norm = vectorNorm(acs.craft_inertial);
	matrixScale(acs.craft_inertial, 1.0/c_i_norm);
	vectorSetXYZ(acs.sv_inertial, 4, 5, 0);
	float s_i_norm = vectorNorm(acs.sv_inertial);
	matrixScale(acs.sv_inertial, 1.0/s_i_norm);
	
	sprintf(transmit, "Rt\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.Rt, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	sprintf(transmit, "C_I\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.craft_inertial, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	sprintf(transmit, "S_I\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.sv_inertial, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	findErrorVectors(&acs);
	
	sprintf(transmit, "Z_err\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.z_err, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	sprintf(transmit, "N_err\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.n_err, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	// TEST 2
	sprintf(transmit, "----- TEST 2 -----\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	acs.Rt = initializeDCM(30, 40, 50);
	vectorSetXYZ(acs.craft_inertial, -3, 2, 3);
	c_i_norm = vectorNorm(acs.craft_inertial);
	matrixScale(acs.craft_inertial, 1.0/c_i_norm);
	vectorSetXYZ(acs.sv_inertial, 4, 1, -5);
	s_i_norm = vectorNorm(acs.sv_inertial);
	matrixScale(acs.sv_inertial, 1.0/s_i_norm);
	
	sprintf(transmit, "Rt\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.Rt, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	sprintf(transmit, "C_I\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.craft_inertial, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	sprintf(transmit, "S_I\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.sv_inertial, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	findErrorVectors(&acs);
	
	sprintf(transmit, "Z_err\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.z_err, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	sprintf(transmit, "N_err\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.n_err, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	
	// Test wdot2rw_pwm()
	// Uses gyro_vector, w_rw, and J's from acs
	sprintf(transmit, "\r\nTesting wdot2rw_pwm()\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	// TEST 1
	sprintf(transmit, "----- TEST 1 -----\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	vectorSetXYZ(acs.gyro_vector, 0.1, 0.2, -0.1);
	vectorSetXYZ(acs.w_rw, -50, -100, 200);
	Matrix wdot_desired = make3x1Vector(0.05, 0.03, 0.04);
	acs.dt = 1;
	
	wdot2rw_pwm(&acs, wdot_desired);
	
	sprintf(transmit, "Reaction wheel PWM\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.rw_PWM, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	
	// Test orientationController()
	sprintf(transmit, "\r\nTesting orientationController()\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	// TEST 1
	sprintf(transmit, "----- TEST 1 -----\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	vectorSetXYZ(acs.gyro_vector, 0, 0, 0);
	vectorSetXYZ(acs.w_rw, 0, 0, 0);
	
	runOrientationController(&acs, 1);
	
	sprintf(transmit, "Reaction wheel PWM\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.rw_PWM, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	// TEST 2
	sprintf(transmit, "----- TEST 2 -----\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	runOrientationController(&acs, 0);
	
	sprintf(transmit, "Reaction wheel PWM\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.rw_PWM, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	// TEST 3
	sprintf(transmit, "----- TEST 3 -----\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	vectorSetXYZ(acs.gyro_vector, 0.03, -0.015, 0.01);
	vectorSetXYZ(acs.w_rw, -30, -150, -190);
	acs.dt = 0.1;
	
	runOrientationController(&acs, 0);
	
	sprintf(transmit, "Reaction wheel PWM\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.rw_PWM, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	
	// Test stabilizationController()
	sprintf(transmit, "\r\nTesting stabilizationController()\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	// TEST 1
	sprintf(transmit, "----- TEST 1 -----\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	runStabilizationController(&acs, acs.err, 1);
	
	sprintf(transmit, "Reaction wheel PWM\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.rw_PWM, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	// TEST 2
	sprintf(transmit, "----- TEST 2 -----\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	runStabilizationController(&acs, acs.err, 0);
	
	sprintf(transmit, "Reaction wheel PWM\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.rw_PWM, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	
	// TEST 3
	sprintf(transmit, "----- TEST 3 -----\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	vectorSetXYZ(acs.gyro_vector, 0.08, 0, -0.025);
	vectorSetXYZ(acs.w_rw, 110, 20, -45);
	
	runStabilizationController(&acs, acs.err, 0);
	
	sprintf(transmit, "Reaction wheel PWM\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
	printMatrix(acs.rw_PWM, transmit);
	HAL_UART_Transmit(&huart2, (uint8_t*)transmit, strlen(transmit), 100);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
