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

 
/* Datatypes -----------------------------------------------------------------*/
typedef enum {
	SV_FOUND,
	SV_NOTFOUND,
	SV_DARK
}SV_Status;

typedef struct {
	// Craft DCM
	Matrix R;
	
	// Sensor vectors
	Matrix gyro_vector;
	Matrix mag_vector;
	Matrix solar_vector;
	
	// Attitude estimation vectors
	Matrix gyro_bias;
	Matrix sv_inertial;
	Matrix mag_inertial;
	
	// Feedback control vectors
	Matrix z_err;
	Matrix n_err;
	Matrix err; // err = z_err + n_err
	
	// Sensors hardware
	I2C_HandleTypeDef* hi2c;
	MovingAvgFilter mag_filter;
	MovingAvgFilter sv_filter;
	uint32_t sv_raw[NUM_SOLAR_PANELS]; // For DMA to use
	
	// Solar vector status
	SV_Status sun_status;
	
	// Satellite dynamic system
	Matrix w_rw; // Reaction wheel speed vector
}ACSType;


/* Initialization Functions --------------------------------------------------*/

/** 
 * @brief  Initializes all the fields in the given ACS struct
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
*/
void initializeACS(ACSType* acs);

#endif
