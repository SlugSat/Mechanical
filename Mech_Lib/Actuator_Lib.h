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
 * @return None
*/
void initActuators(TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_rpm);

void rw_init(uint32_t rw_fwd_rev_pin, GPIO_TypeDef* rw_fwd_rev_port, uint32_t rw_brake_pin, GPIO_TypeDef* rw_brake_port);

void tr_init(uint32_t tr_fwd_rev_pin, GPIO_TypeDef* tr_fwd_rev_port, uint32_t tr_enable_pin, GPIO_TypeDef* tr_enable_port);

void rw_get_speed(float *speed);

void rw_set_speed(float pwm, uint8_t brake);

void tr_set_speed(float pwm);

#endif /* ACTUATOR_LIB_H */
