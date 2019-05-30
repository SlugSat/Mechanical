/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ENABLE_42 // Enables 42 integration
#define ENABLE_FRAM // Enables writing to shared FRAM
#define ENABLE_ACTUATORS // Enables code to control flat-sat prototype actuators
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI_FRAM_IN2_Pin GPIO_PIN_10
#define SPI_FRAM_IN2_GPIO_Port GPIOB
#define RW_FWD_REV_Pin GPIO_PIN_11
#define RW_FWD_REV_GPIO_Port GPIOB
#define HALL_EFFECT_IN_Pin GPIO_PIN_12
#define HALL_EFFECT_IN_GPIO_Port GPIOB
#define RW_PWM_Pin GPIO_PIN_13
#define RW_PWM_GPIO_Port GPIOB
#define TR_PWM_Pin GPIO_PIN_14
#define TR_PWM_GPIO_Port GPIOB
#define SPI_FRAM_IN1_Pin GPIO_PIN_8
#define SPI_FRAM_IN1_GPIO_Port GPIOA
#define SPI_FRAM_LOCK_Pin GPIO_PIN_9
#define SPI_FRAM_LOCK_GPIO_Port GPIOA
#define SPI_FRAM_CS_Pin GPIO_PIN_6
#define SPI_FRAM_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
