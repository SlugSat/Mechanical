/**
  ******************************************************************************
  * @file           ACSStateMachine.c
  * @brief          Runs the logic for the ACS
  ******************************************************************************
  * 
	* 
	* Created by Galen Savidge. Edited 5/9/2019.
  ******************************************************************************
  */

// Header file
#include <ACSStateMachine.h>

// ACS modules
#include <ACS.h>
#include <STM32SerialCommunication.h>
#include <InertialVectors.h>
#include <AttitudeEstimation.h>
#include <FeedbackControl.h>

// Standard libraries
#include <math.h>


// Thresholds used for state machine transitions
#define DETUMBLE_LOW_THRESHOLD 0.00872665	// 0.5 deg/s in rad/s
#define DETUBMLE_HIGH_THRESHOLD 0.05
#define STABLE_ATTITUDE_THRESHOLD 0.015 	// Rad/s^2; will have to be updated for real sensors
#define POINT_ERROR_THRESHOLD_HIGH 20			// Degrees
#define POINT_ERROR_THRESHOLD_LOW 10


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


void runACS(UART_HandleTypeDef* huart) {
	/***** INITIALIZE ACS *****/
	ACSType acs;
	initializeACS(&acs);
	initializeACSSerial(&acs, huart); // Only necessary to communicate with 42
	
	ACSState state = WAIT_FOR_ATTITUDE;
	int first_step = 1;
	
	// Variables to hold values that are important for state transitions
	uint8_t acs_enable = 1;			// Temporary bool
	float gyro_vector_norm;			// Rad/s
	float attitude_est_stable_counter = 0;
	
	char prnt[300]; // String buffer to print to 42
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		// Print to terminal
		sprintf(prnt, "State -- %s\nPointing error: %6.2f [deg]\nCraft rotational rate: %6.2f [deg/s]\nGyro bias dot: %6.2f [rad/s^2]", 
				state_names[state], acs.pointing_err, 180*gyro_vector_norm/PI, acs.gyro_bias_dot_norm);
		
		
		/***** READ/WRITE TO 42 *****/
		STM32SerialHandshake(huart);
		readSensorsFromSerial(&acs);
		sendActuatorsToSerial(&acs);
		STM32SerialSendString(huart, prnt);
		
		
		/***** RUN ACS SUBROUTINES *****/
		gyro_vector_norm = vectorNorm(acs.gyro_vector);
		
		if(state == DETUMBLE) {
			// Read gyro here
			
			// Run bdot controller
			runBdotController(&acs);
		}
		
		if(state == WAIT_FOR_ATTITUDE || state == REORIENT || state == STABILIZE) {
			// Read IMU (mag and gyro) here
			
			// Run attitude estimation
			findSunInertial(&acs);
			findMagInertial(&acs);
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
			runStabilizationController(&acs, acs.err, first_step);
			first_step = 0;
		}
		
		
		/***** RUN STATE MACHINE *****/
		ACSState next_state = state;
		
		// Check for state transitions
		switch(state) {
			case WAIT_FOR_ENABLE:
				if(acs_enable){
					next_state = DETUMBLE;
				}
				break;
			
			case DETUMBLE:
				if(gyro_vector_norm < DETUMBLE_LOW_THRESHOLD) {
					next_state = WAIT_FOR_ATTITUDE;
				}
				break;
				
			case WAIT_FOR_ATTITUDE:
				// Count iterations where attitude is stable
				if(fabsf(acs.gyro_bias_dot_norm) < STABLE_ATTITUDE_THRESHOLD) {
					attitude_est_stable_counter++;
				}
				else {
					attitude_est_stable_counter = 0;
				}
				
				// Transition when we hit 10 in a row
				if(attitude_est_stable_counter >= 10) {
					next_state = REORIENT;
				}
				break;
				
			case REORIENT:
				if(acs.pointing_err < POINT_ERROR_THRESHOLD_LOW) {
					next_state = STABILIZE;
				}
				break;
				
			case STABILIZE:
				if(acs.pointing_err > POINT_ERROR_THRESHOLD_HIGH) {
					next_state = REORIENT;
				}
				break;
				
			default:
				break;
		}
		
		// State transition logic
		if(next_state != state)
		{
			// Exit events
			switch(state) {
			case WAIT_FOR_ENABLE:
				break;
			
			case DETUMBLE:
				break;
				
			case WAIT_FOR_ATTITUDE:
				break;
				
			case REORIENT:
				break;
				
			case STABILIZE:
				break;
				
			default:
				break;
			}
			
			// Switch states
			state = next_state;
			
			// Entry events
			switch(state) {
			case WAIT_FOR_ENABLE:
				break;
			
			case DETUMBLE:
				break;
				
			case WAIT_FOR_ATTITUDE:
				attitude_est_stable_counter = 0;
				break;
				
			case REORIENT:
				first_step = 1;
				break;
				
			case STABILIZE:
				first_step = 1;
				break;
				
			default:
				break;
			}
		}
	}
}
