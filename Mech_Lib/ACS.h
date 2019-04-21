/**
  ******************************************************************************
  * @file           : AttitudeEstimation.h
  * @brief          : Header file for the Attitude Control System (ACS).
  ******************************************************************************
  ** 
	* 
	* Created by Galen Savidge. Edited 4/19/2019.
  ******************************************************************************
  */

#ifndef ACS_H
#define ACS_H

/* Includes ------------------------------------------------------------------*/
#include <Matrix.h>
#include <DigitalFilters.h>
#include <main.h>
#include <stdint.h>


/* Constants -----------------------------------------------------------------*/
#define NUM_SOLAR_PANELS 6
#define PI 3.14159265359
#define MAX_MOMENT 2


 
/* Datatypes -----------------------------------------------------------------*/
typedef enum {
	SV_FOUND,
	SV_NOTFOUND,
	SV_DARK
}SV_Status;

typedef struct {
	// Craft DCM
	Matrix R;
	Matrix Rt;
	
	// Sensor vectors (body frame)
	Matrix gyro_vector;
	Matrix mag_vector;
	Matrix solar_vector;
	
	// Attitude estimation vectors
	Matrix gyro_bias;
	
	// Inertial vectors (ecliptic frame)
	Matrix sv_inertial; // Found using the Julian date
	Matrix mag_inertial; // From IGRF
	Matrix craft_inertial; // From SGP4
	
	// Feedback control error vectors (body frame)
	Matrix z_err;
	Matrix n_err;
	Matrix err; // err = z_err + n_err
	
	// Sensor hardware
	MovingAvgFilter mag_filter;
	MovingAvgFilter sv_filter;
	uint32_t sv_raw[NUM_SOLAR_PANELS]; // For DMA to use
	
	// Solar vector status
	SV_Status sun_status;
	
	// Satellite dynamic system
	Matrix w_rw; // Reaction wheel speed vector (body frame)
	Matrix J_rw; // Reaction wheel inertia matrix
	Matrix A_rw; // Reaction wheel projection onto body axes
	Matrix J_body; // Craft inertia matrix
	Matrix J_body_inv; // Inverse of J_body
}ACSType;


/* Initialization Functions --------------------------------------------------*/

/** 
 * @brief  Initializes all the fields in the given ACS struct
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
*/
void initializeACS(ACSType* acs);

/* Serial Communication Functions ---------------------------------------------*/

/** 
 * @brief  Reads sensor data from serial and updates acs
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
*/
void readSensorsFromSerial(ACSType* acs, UART_HandleTypeDef* huart); // To-do
#endif
