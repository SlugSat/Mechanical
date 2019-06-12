/*
  ******************************************************************************
  * @file           ACSStateMachine.c
  * @brief          Runs the logic for the ACS
  ******************************************************************************
  * Created by Galen Savidge. Edited 5/11/2019.
  ******************************************************************************
  */

// Header file
#include <ACSStateMachine.h>

// ACS modules
#include <ACS.h>
#include <InertialVectors.h>
#include <AttitudeEstimation.h>
#include <FeedbackControl.h>

// 42
#ifdef ENABLE_42
#include <STM32SerialCommunication.h>
#endif

// FRAM
#ifdef ENABLE_FRAM
#include <SPI_FRAM.h>
#endif

// Standard libraries
#include <math.h>


// Thresholds used for state machine transitions
#define DETUMBLE_LOW_THRESHOLD 0.00872665			// 0.5 deg/s in rad/s
#define DETUBMLE_HIGH_THRESHOLD 0.05
#define STABLE_ATTITUDE_LOW_THRESHOLD 4e-5		// Rad/s^2; will have to be updated for real sensors
#define STABLE_ATTITUDE_HIGH_THRESHOLD 1e-4
#define POINT_ERROR_HIGH_THRESHOLD 12					// Degrees
#define POINT_ERROR_LOW_THRESHOLD 7
#define SUN_ANGLE_HIGH_THRESHOLD 32						// Degrees
#define SUN_ANGLE_LOW_THRESHOLD 30

#define INERTIAL_UPDATE_RATE 1 								// Seconds between intertial vector updates

#define RPM_2_RADS 0.104719755				// Conversion between RPM and rad/s

// Serial device handles
#ifdef ENABLE_42
static UART_HandleTypeDef* huart;

char state_names[][30] = {
		"Default", 
		"Wait for Enable", 
		"Detumble", 
		"Wait for Attitude", 
		"Reorient",
		"Stabilize (Ignore Sun)",
		"Stabilize"
};

char sun_status_names[][20] = {
	"SV Found",
	"SV Invalid",
	"Eclipse"
};

void setUartHandle(UART_HandleTypeDef* uart) {
	huart = uart;
}
#endif

#ifdef ENABLE_FRAM
#define FRAM_TIMEOUT 100

static SPI_HandleTypeDef* hspi;

void setSpiHandle(SPI_HandleTypeDef* spi) {
	hspi = spi;
}
#endif


void runACS(void) {
	/***** INITIALIZE ACS *****/
	ACSType acs;
	initializeACS(&acs);
	
	ACSState state = WAIT_FOR_ENABLE;
	int first_step = 1, reset_integrator = 1;
	float last_inertial_update_time = -INFINITY;	// In seconds
	
	// Variables to hold values that are important for state transitions
	uint8_t acs_enable = 1;			// Temporary bool
	float gyro_norm = 0, w_norm = 0;		// Rad/s
	float attitude_est_stable_counter = 0;

	//Matrix zero_vector = make3x1Vector(0, 0, 0);
	
	#ifdef ENABLE_42
	initializeACSSerial(huart);
	char prnt[300]; // String buffer to print to 42
	#endif

	#ifdef ENABLE_FRAM
	SPI_FRAM_Init(hspi);
	#endif

	#ifdef ENABLE_ACTUATORS
	float rw_speed = 0;	// Speed of the reaction wheel prototype, rad/s
	#endif
	
	while (1) {
		#ifdef ENABLE_42
		/***** READ/WRITE TO 42 *****/
		syncWith42(&acs);
		#endif
		
		
		/***** RUN ACS SUBROUTINES *****/
		// Read reaction wheel speeds
		#ifdef ENABLE_ACTUATORS
		rw_get_speed(&rw_speed);
		rw_speed = RPM_2_RADS*rw_speed;
		#endif

		// Read solar vectors here to get sun state
		
		// Update ACS fields
		J2000_2_LongLatAlt(acs.craft_j2000, acs.julian_date, &acs.longitude, &acs.latitude, &acs.altitude);
		matrixSubtract(acs.gyro_vector, acs.gyro_bias, acs.w);
		
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
		
		if(state == WAIT_FOR_ATTITUDE) {
			// Use reaction wheels to stabilize
			//wdot2rw_pwm(&acs, zero_vector);
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
			runStabilizationController(&acs, acs.z_err, first_step, reset_integrator);
			first_step = 0;
			reset_integrator = 0;
		}
		
		if(state == STABILIZE) {
			// Run momentum dumping
			findErrorVectors(&acs);
			runStabilizationController(&acs, acs.err, first_step, reset_integrator);
			first_step = 0;
			reset_integrator = 0;
		}
		

		/***** SET ACTUATOR OUTPUTS *****/
		#ifdef ENABLE_ACTUATORS
		rw_set_speed(matrixGetElement(acs.rw_PWM, 1, 1), acs.rw_brake[0]);
		tr_set_speed(matrixGetElement(acs.tr_PWM, 1, 1));
		#endif
		
		
		/***** RUN STATE MACHINE *****/
		/*
		 * See the ACS State Machine (figure x, page x of the Year 3 Sping Report).
		 *
		 * State transition checks at the top of the code have low priority,
		 * checks at the bottom have high priority.
		 */
		ACSState next_state = state;
		
		gyro_norm = vectorNorm(acs.gyro_vector);
		w_norm = vectorNorm(acs.w);
		
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
				if(gyro_norm < DETUMBLE_LOW_THRESHOLD) {
					next_state = WAIT_FOR_ATTITUDE;
				}
				break;
				
			case WAIT_FOR_ATTITUDE:
				// Count iterations where attitude is stable
				if(acs.gyro_bias_dot_norm < STABLE_ATTITUDE_LOW_THRESHOLD && acs.sun_status == SV_FOUND) {
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
					reset_integrator = 1;
				}
				break;
				
			case STABILIZE_NO_SUN:
				if(acs.sun_status != SV_DARK && acs.zb_sun_angle > SUN_ANGLE_HIGH_THRESHOLD) {
					next_state = STABILIZE;
				}
				break;
				
			// Stabilize has no forward transition
				
			default:
				break;
		}
		
		// Reverse state transitions
		if(state >= STABILIZE) {
			if(acs.sun_status == SV_DARK || acs.zb_sun_angle < SUN_ANGLE_LOW_THRESHOLD) { // In eclipse or right under the sun
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
			
			// Add check for 
		}
		
		if(state >= WAIT_FOR_ATTITUDE) {
			if(w_norm > DETUBMLE_HIGH_THRESHOLD) { // We are tumbling again
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
				vectorSetXYZ(acs.rw_PWM, 100, 100, 100);
				acs.rw_brake[0] = 1;
				acs.rw_brake[1] = 1;
				acs.rw_brake[2] = 1;
				break;
				
			case WAIT_FOR_ATTITUDE:
				attitude_est_stable_counter = 0;
				vectorSetXYZ(acs.rw_PWM, 0, 0, 0);
				acs.rw_brake[0] = 0;
				acs.rw_brake[1] = 0;
				acs.rw_brake[2] = 0;
				vectorSetXYZ(acs.tr_PWM, 0, 0, 0);
				break;
				
			case REORIENT:
				first_step = 1;
				reset_integrator = 1;
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
		
		
		#ifdef ENABLE_FRAM
		/***** READ/WRITE TO FRAM *****/
		SPI_FRAM_Write(hspi, SPI_FRAM_LATITUDE_ADDR, (uint8_t*)&acs.latitude, 4, 0, FRAM_TIMEOUT);
		SPI_FRAM_Write(hspi, SPI_FRAM_LONGITUDE_ADDR, (uint8_t*)&acs.longitude, 4, 0, FRAM_TIMEOUT);
		SPI_FRAM_Write(hspi, SPI_FRAM_ALTITUDE_ADDR, (uint8_t*)&acs.altitude, 4, 0, FRAM_TIMEOUT);
		SPI_FRAM_Write(hspi, SPI_FRAM_TIME_ADDR, (uint8_t*)&acs.julian_date, 8, 0, FRAM_TIMEOUT);
		SPI_FRAM_Write(hspi, SPI_FRAM_SOLAR_VECTOR_ADDR, (uint8_t*)&acs.sun_status, 1, 0, FRAM_TIMEOUT);
		SPI_FRAM_Write(hspi, SPI_FRAM_MECH_STATE_ADDR, (uint8_t*)&state, 1, 0, FRAM_TIMEOUT);
		#endif
		
		
		#ifdef ENABLE_42
		// ***** PRINT TO 42 TERMINAL *****/
		sprintf(prnt, "State -- %s\n", state_names[state]);
		printTo42(prnt);
		
		if(state == DETUMBLE) {
			sprintf(prnt, "Gyro reading: %6.2f [deg/s]\nGyro w/ bias: %6.2f [deg/s]\n", 
				180*gyro_norm/PI, 180*w_norm/PI);
			printTo42(prnt);
		}
		else if(state == WAIT_FOR_ATTITUDE || state == REORIENT || state == STABILIZE_NO_SUN || state == STABILIZE) {
			sprintf(prnt, "Pointing error: %6.2f [deg]\nCraft rotational rate: %6.2f [deg/s]\nSun status -- %s\nGyro bias dot: %8.6f [rad/s^2]\n",
					acs.pointing_err, 180*w_norm/PI, sun_status_names[acs.sun_status], acs.gyro_bias_dot_norm);
			printTo42(prnt);
			
			sprintf(prnt, "Angle to sun: %6.2f [deg]\n", acs.zb_sun_angle);
			printTo42(prnt);
		}

		#ifdef ENABLE_ACTUATORS
		sprintf(prnt, "RW speed: %5.2f [rad/s]", rw_speed);
		printTo42(prnt);
		#endif
		#endif
	}
}
