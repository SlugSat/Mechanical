#include "Actuator_Lib.h"

#define N_SAMPLES 100.0
#define TIM_RPM_CLK 500000.0

static TIM_HandleTypeDef *ht_rpm, *ht_pwm;
static uint16_t time_val;
static uint8_t IC_flag = 0;

static float avg_speed = 0;

void initActuators(TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_rpm)
{
	// initialize local module level timer handlers
	ht_rpm = htim_rpm; // timer handler for rpm calc
	ht_pwm = htim_pwm; // timer handler for pwm output
	
	// start timer for PWM
	HAL_TIM_Base_Start(ht_pwm);
	
	// start interrupt handler
	HAL_TIM_Base_Start_IT(ht_rpm);
	HAL_TIM_IC_Start_IT(ht_rpm, TIM_CHANNEL_1);
	
	// set PWM signal frequency
	PWM_Set_Frequency(ht_pwm, 100);
	
	// starts PWM signal generation
	if (HAL_TIM_PWM_Start(ht_pwm, TIM_CHANNEL_1) != HAL_OK) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}
	
	// start timer for RPM calculations
	ht_rpm->Instance->CCER |= 1;
	HAL_TIM_Base_Start(ht_rpm);
}

void rw_get_speed (float *speed)
{
		// local variable to store pulse time
		float xor_pulse_time;
		
		// time for rotation (us) assuming pulses are equidistant
		xor_pulse_time = 6.0 * time_val; 	// time to rotate in us
		xor_pulse_time = xor_pulse_time 									// (us/rev)
										 * (1.0/TIM_RPM_CLK) 							// (s/us)
										 * (1.0/60);											// (min/s)
			
		// calculate speed (RPM)
		*speed = (1.0/xor_pulse_time)*3.0;

		// calculate average speed (moving average)
		avg_speed -= avg_speed/N_SAMPLES;
		avg_speed += *speed/N_SAMPLES;
		
		*speed = avg_speed;
}

void rw_set_speed (float percent_speed)
{
	// set duty cycle accordingly
	PWM_Set_Duty_Cycle(ht_pwm, percent_speed, RW_PWM_CHANNEL);
}

void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim) {
	time_val = htim->Instance->CCR1;
	IC_flag = 1;
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
	if (!IC_flag) {
		time_val = 0;
	}
	IC_flag = 0;
}
