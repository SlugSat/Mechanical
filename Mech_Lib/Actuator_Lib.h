#ifndef ACTUATOR_LIB_H
#define ACTUATOR_LIB_H

#include "main.h"
#include "PWM_Library.h"


void rw_init(TIM_HandleTypeDef *htim_pulse, 	// PWM timer
						 TIM_HandleTypeDef *htim_speed);	// RPM timer

void rw_get_speed (float *speed);

void rw_set_speed (float percent_speed);

void rw_get_accel(float *accel);

#endif
