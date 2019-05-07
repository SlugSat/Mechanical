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
// ACS modules
#include <ACS.h>
#include <STM32SerialCommunication.h>
#include <InertialVectors.h>
#include <AttitudeEstimation.h>
#include <FeedbackControl.h>

// C libraries
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	DEFAULT = 0,
	WAIT_FOR_ENABLE,
	DETUMBLE,
	WAIT_FOR_ATTITUDE,
	REORIENT,
	STABILIZE
}ACSState;

char state_names[][20] = {
		"Default", 
		"Wait for Enable", 
		"Detumble", 
		"Wait for Attitude", 
		"Reorient", 
		"Stabilize"
};
	
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DETUMBLE_THRESHOLD 0.1 					// Rad/s
#define STABLE_ATTITUDE_THRESHOLD 0.1 	// Rad/s^2
#define POINT_ERROR_THRESHOLD_HIGH 20		// Degrees
#define POINT_ERROR_THRESHOLD_LOW 10

#define GYRO_READ_TIME_MS 5
#define IMU_READ_TIME_MS 9
#define ATTITUDE_EST_TIME_MS 11
#define FEEDBACK_CONTROL_TIME_MS 15
#define BDOT_TIME_MS 10
#define MOMENTUM_DUMP_TIME_MS 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ACSState state = REORIENT;
ACSType acs;

// Temporary variables to hold values that are important for state transitions
uint8_t acs_enable = 1;					// Bool
float gyro_vector_norm = 0;			// Rad/s
float gyro_vector_norm_dot = 0;	// Rad/s^2
float pointing_error = 3;				// Degrees

float v; // Variable used by delay function

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// Simulates processor load by doing floating point operations for the specified
// number of ms
void LoadProcessor(uint8_t time_ms);

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
	
	/***** INITIALIZE ACS *****/
	initializeACS(&acs);
	initializeACSSerial(&acs, &huart2); // Only necessary to communicate with 42
	
	int first_step = 1;
	
	char prnt[300]; // String buffer to print to 42
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		// Print to terminal
		sprintf(prnt, "State -- %s\nPointing error: %6.2f", state_names[state], acs.pointing_err);
		
		
		/***** READ/WRITE TO 42 *****/
		STM32SerialHandshake(&huart2);
		readSensorsFromSerial(&acs);
		sendActuatorsToSerial(&acs);
		STM32SerialSendString(&huart2, prnt);
		
		// Update inertial vectors (this should be optimized)
		findSunInertial(&acs);
		findMagInertial(&acs);
		
		
		/***** RUN ACS SUBROUTINES *****/
		if(state == DETUMBLE) {
			// Read gyro here
			
			// Run bdot controller
			runBdotController(&acs);
		}
		
		if(state == WAIT_FOR_ATTITUDE || state == REORIENT || state == STABILIZE) {
			// Read IMU (mag and gyro) here
			
			// Run attitude estimation
			updateAttitudeEstimate(&acs);
		}
		
		if(state == REORIENT || state == STABILIZE) {
			// Run feedback controller
			findErrorVectors(&acs);
			runOrientationController(&acs, first_step);
			first_step = 0;
		}
		
		if(state == STABILIZE) {
			// Run momentum dumping
			findErrorVectors(&acs);
			runStabilizationController(&acs, first_step);
			first_step = 0;
		}
		
		
		/***** RUN STATE MACHINE *****/
		ACSState next_state = state;
		
		switch(state) {
			case WAIT_FOR_ENABLE:
				if(acs_enable){
					next_state = DETUMBLE;
				}
				break;
			
			case DETUMBLE:
				if(gyro_vector_norm < DETUMBLE_THRESHOLD) {
					next_state = WAIT_FOR_ATTITUDE;
				}
				break;
				
			case WAIT_FOR_ATTITUDE:
				if(fabsf(gyro_vector_norm_dot) < STABLE_ATTITUDE_THRESHOLD) {
					next_state = REORIENT;
					first_step = 1;
				}
				break;
				
			case REORIENT:
				if(acs.pointing_err < POINT_ERROR_THRESHOLD_LOW) {
					next_state = STABILIZE;
					first_step = 1;
				}
				break;
				
			case STABILIZE:
				if(acs.pointing_err > POINT_ERROR_THRESHOLD_HIGH) {
					next_state = REORIENT;
					first_step = 1;
				}
				break;
				
			default:
				break;
		}
		
		if(next_state != state)
		{
			state = next_state;
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

void LoadProcessor(uint8_t time_ms)
{
	uint16_t init_time = TIM2->CNT;
	uint16_t delay_time_10us = time_ms*100;
	
	v = (float)rand()/(float)rand();
	
	while((uint16_t)(TIM2->CNT - init_time) < delay_time_10us) {
		v = v*(float)rand()/(float)rand();
	}
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
