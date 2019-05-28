#include "Actuator_Lib.h"

#define N_SAMPLES 100.0
#define TIM_RPM_CLK 4e6

static TIM_HandleTypeDef *ht_rpm, *ht_pwm;
static volatile uint32_t FWD_REV_Pin;
static uint16_t time_val;
static uint8_t IC_flag = 0;

static float avg_speed = 0;

void initActuators(TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_rpm, volatile uint32_t DIRECTION_Pin)
{
    // initialize local module level timer handlers
    ht_rpm = htim_rpm; // timer handler for rpm calc
    ht_pwm = htim_pwm; // timer handler for pwm output
    
    // start timer for PWM
    HAL_TIM_Base_Start(ht_pwm);
    
    // start interrupt handler
    HAL_TIM_Base_Start_IT(ht_rpm);
    HAL_TIM_IC_Start_IT(ht_rpm, RW_PWM_CHANNEL);
    
    // set PWM signal frequency
    PWM_Set_Frequency(ht_pwm, 100);
    
    // starts PWM signal generation
    HAL_TIM_PWM_Start(ht_pwm, RW_PWM_CHANNEL);
    
    // start timer for RPM calculations
    ht_rpm->Instance->CCER |= 1;
    HAL_TIM_Base_Start(ht_rpm);

    // save FWD_REV_Pin
    FWD_REV_Pin = DIRECTION_Pin;
}

void rw_get_speed (float *speed)
{
    /* CODE TO CALCULATE RPM USING XOR OF 3 HALL EFF SENSORS
	// local variable to store pulse time
	float xor_pulse_time;

	// time for rotation (us) assuming pulses are equidistant
	xor_pulse_time = 6.0 * time_val;        // time to rotate in us
	xor_pulse_time = xor_pulse_time         // (us/rev)
					 * (1.0/TIM_RPM_CLK)    // (s/us)
					 * (1.0/60);            // (min/s)

	// calculate speed (RPM)
	*speed = (1.0/xor_pulse_time);*/

	// calculate speed (RPM)
	*speed = time_val               // (us/rev)
			 * (1.0/TIM_RPM_CLK)    // (s/us)
			 * (1.0/60);            // (min/s)

	// calculate average speed (moving average)
	avg_speed -= avg_speed/N_SAMPLES;
	avg_speed += *speed/N_SAMPLES;

	*speed = avg_speed;
}

void rw_set_speed (float percent_speed)
{
    // check direction
    if (percent_speed > 0) {
        HAL_GPIO_WritePin(GPIOB, FWD_REV_Pin, GPIO_PIN_RESET);
    } else {
        percent_speed *= -1;
        HAL_GPIO_WritePin(GPIOB, FWD_REV_Pin, GPIO_PIN_SET);
    }

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
