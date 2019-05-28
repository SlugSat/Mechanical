/*
  ******************************************************************************
  * @file           Actuator_Lib.h
  * @brief
  ******************************************************************************
  * Created by Alex Martinez. Edited 5/27/2019.
  ******************************************************************************
  */

#ifndef ACTUATOR_LIB_H
#define ACTUATOR_LIB_H

#include "main.h"
#include "PWM_Library.h"

#define RW_PWM_CHANNEL TIM_CHANNEL_1
#define TR_PWM_CHANNEL TIM_CHANNEL_2

/**
 * @brief  Sets up timers for PWM generation and reaction wheel speed measurement
 * @param  htim_pwm: Handle to the PWM timer, which should be running at 4 MHz
 * @param  htim_rpm: Handle to the RPM timer
 * @param  DIRECTION_Pin: Forward/Reverse Pin, which should be a GPIOB
 * @return None
*/
void initActuators(TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_rpm, volatile uint32_t DIRECTION_Pin);

void rw_get_speed (float *speed);

void rw_set_speed (float percent_speed);

void rw_get_accel(float *accel);

#endif /* ACTUATOR_LIB_H */
