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
#define DETUMBLE_LOW_THRESHOLD 0.00872665			// 0.5 deg/s in rad/s
#define DETUBMLE_HIGH_THRESHOLD 0.05
#define STABLE_ATTITUDE_LOW_THRESHOLD 0.015 	// Rad/s^2; will have to be updated for real sensors
#define STABLE_ATTITUDE_HIGH_THRESHOLD 0.05
#define POINT_ERROR_HIGH_THRESHOLD 20					// Degrees
#define POINT_ERROR_LOW_THRESHOLD 10


#define INERTIAL_UPDATE_RATE 10 							// Seconds between intertial vector updates


typedef enum {
	DEFAULT = 0,
	WAIT_FOR_ENABLE,
	DETUMBLE,
	WAIT_FOR_ATTITUDE,
	REORIENT,
	STABILIZE_NO_SUN,
	STABILIZE
}ACSState;

char state_names[][30] = {
		"Default", 
		"Wait for Enable", 
		"Detumble", 
		"Wait for Attitude", 
		"Reorient",
		"Stabilize (Ignore Sun)",
		"Stabilize"
};


void runACS(UART_HandleTypeDef* huart) {
	/***** INITIALIZE ACS *****/
	ACSType acs;
	initializeACS(&acs);
	initializeACSSerial(&acs, huart); // Only necessary to communicate with 42
	
	ACSState state = DEFAULT;
	int first_step = 1;
	float last_inertial_update_time = -INFINITY;	// In seconds
	
	// Variables to hold values that are important for state transitions
	uint8_t acs_enable = 1;			// Temporary bool
	float gyro_vector_norm;			// Rad/s
	float attitude_est_stable_counter = 0;
	
	char prnt[300]; // String buffer to print to 42
	
  while (1) {
		// Print to terminal
		sprintf(prnt, "State -- %s\nPointing error: %6.2f [deg]\nCraft rotational rate: %6.2f [deg/s]\nGyro bias dot: %6.2f [rad/s^2]", 
				state_names[state], acs.pointing_err, 180*gyro_vector_norm/PI, acs.gyro_bias_dot_norm);
		
		
		/***** READ/WRITE TO 42 *****/
		STM32SerialHandshake(huart);
		readSensorsFromSerial(&acs);
		sendActuatorsToSerial(&acs);
		STM32SerialSendString(huart, prnt);
		
		
		/***** RUN ACS SUBROUTINES *****/
		// Read solar vectors here to get sun state
		
		if(state == DETUMBLE) {
			// Read gyro here
			
			// Run bdot controller
			runBdotController(&acs);
		}
		
		if(state == WAIT_FOR_ATTITUDE || state == REORIENT || state == STABILIZE_NO_SUN || state == STABILIZE) {
			// Read IMU (mag and gyro) here
			
			// Update inertial vectors
			if(acs.t - last_inertial_update_time >= INERTIAL_UPDATE_RATE) {
				findSunInertial(&acs);
				findMagInertial(&acs);
				last_inertial_update_time = acs.t;
			}
			
			// Run attitude estimation
			updateAttitudeEstimate(&acs);
		}
		
		if(state == REORIENT) {
			// Run feedback controller
			findErrorVectors(&acs);
			runOrientationController(&acs, first_step);
			first_step = 0;
		}
		
		if(state == STABILIZE_NO_SUN) {
			// Run momentum dumping
			findErrorVectors(&acs);
			runStabilizationController(&acs, acs.z_err, first_step);
			first_step = 0;
		}
		
		if(state == STABILIZE) {
			// Run momentum dumping
			findErrorVectors(&acs);
			runStabilizationController(&acs, acs.err, first_step);
			first_step = 0;
		}
		
		
		/***** RUN STATE MACHINE *****/
		/**
		 * See ACS State Machine (figure x, page x of the Year 3 Sping Report).
		 *
		 * State transition checks at the top of the code have low priority,
		 * checks at the bottom have high priority.
		 */
		ACSState next_state = state;
		
		gyro_vector_norm = vectorNorm(acs.gyro_vector);
		
		// Check forward state transitions
		switch(state) {
			case DEFAULT:
				next_state = DETUMBLE; // Initial state transition
				break;
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
				if(acs.gyro_bias_dot_norm < STABLE_ATTITUDE_LOW_THRESHOLD) {
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
				if(acs.pointing_err < POINT_ERROR_LOW_THRESHOLD) {
					next_state = STABILIZE_NO_SUN;
				}
				break;
				
			case STABILIZE_NO_SUN:
				if(acs.sun_status != SV_DARK) {
					next_state = STABILIZE;
				}
				break;
				
			// Stabilize has no forward transition
				
			default:
				break;
		}
		
		// Reverse state transitions
		if(state >= STABILIZE) {
			if(acs.sun_status == SV_DARK) { // In eclipse
				next_state = STABILIZE_NO_SUN;
			}
		}
		
		if(state >= STABILIZE_NO_SUN) {
			if(acs.pointing_err > POINT_ERROR_HIGH_THRESHOLD) { // We are no longer nadir pointing
				next_state = REORIENT;
			}
		}
		
		if(state >= REORIENT) {
			if(acs.gyro_bias_dot_norm > STABLE_ATTITUDE_HIGH_THRESHOLD) { // Attitude estimate is no longer accurate
				next_state = WAIT_FOR_ATTITUDE;
			}
		}
		
		if(state >= WAIT_FOR_ATTITUDE) {
			if(gyro_vector_norm > DETUBMLE_HIGH_THRESHOLD) { // We are tumbling again
				next_state = DETUMBLE;
			}
		}
		
		
		// State transition logic
		if(next_state != state)
		{
			// Exit events
			switch(state) {
			case WAIT_FOR_ENABLE:
				break;
			
			case DETUMBLE:
				// Turn off torque rods
				vectorSetXYZ(acs.tr_PWM, 0, 0, 0);
				break;
				
			case WAIT_FOR_ATTITUDE:
				break;
				
			case REORIENT:
				break;
				
			case STABILIZE_NO_SUN:
				// Turn off torque rods
				vectorSetXYZ(acs.tr_PWM, 0, 0, 0);
				break;
			
			case STABILIZE:
				// Turn off torque rods
				vectorSetXYZ(acs.tr_PWM, 0, 0, 0);
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
				// Set reaction wheels to default state
				vectorSetXYZ(acs.rw_PWM, 50, 50, 50);
				break;
				
			case WAIT_FOR_ATTITUDE:
				attitude_est_stable_counter = 0;
				break;
				
			case REORIENT:
				first_step = 1;
				break;
			
			case STABILIZE_NO_SUN:
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