/**
  ******************************************************************************
  * @file           : ACS.h
  * @brief          : Header file for the Attitude Control System (ACS).
  ******************************************************************************
	* Created by Galen Savidge. Edited 5/11/2019.
  ******************************************************************************
  */

#ifndef ACS_H
#define ACS_H

/* Includes ------------------------------------------------------------------*/
#include <Matrix.h>
#include <DigitalFilters.h>
#include <ReferenceFrames.h>
#include <main.h>
#include <stdint.h>
#include <string.h>


/* Constants -----------------------------------------------------------------*/
#define NUM_SOLAR_PANELS 6

 
/* Datatypes -----------------------------------------------------------------*/
typedef enum {
	SV_FOUND,
	SV_NOTFOUND,
	SV_DARK
}SV_Status;

typedef struct {
	// Current time
	double julian_date;
	float t; // In seconds
	float dt;
	
	// Craft DCM
	Matrix R;
	Matrix Rt;
	
	// Sensor vectors (body frame)
	Matrix gyro_vector;
	Matrix mag_vector;
	Matrix solar_vector;
	
	// Attitude estimation vectors
	Matrix gyro_bias;
	float gyro_bias_dot_norm;
	
	// Inertial vectors (ecliptic frame)
	Matrix sv_inertial; // Found using the Julian date
	Matrix mag_inertial; // From IGRF
	Matrix craft_inertial; // Normalized
	
	// Craft position in different frames
	Matrix craft_j2000; // From SGP4 or 42
	float longitude, latitude, altitude; // In degrees, degrees, meters wrt prime meridian
	
	// Feedback control error vectors (body frame)
	Matrix z_err;
	Matrix n_err;
	Matrix err; // err = z_err + n_err
	
	// Error scalars (degrees)
	float pointing_err;
	float zb_sun_angle;
	
	// Sensor hardware
	MovingAvgFilter mag_filter;
	MovingAvgFilter sv_filter;
	uint32_t sv_raw[NUM_SOLAR_PANELS]; // For DMA to use
	
	// Solar vector status
	SV_Status sun_status;
	
	// Satellite dynamic system
	Matrix w_rw; // Reaction wheel speed vector (body frame)
	Matrix J_rw; // Reaction wheel inertia matrix
	Matrix J_rw_inv; // Inverse of J_rw
	Matrix A_rw; // Reaction wheel projection onto body axes
	Matrix J_body; // Craft inertia matrix
	Matrix J_body_inv; // Inverse of J_body
	
	// Actuator PWMs
	Matrix rw_PWM;
	Matrix tr_PWM;
}ACSType;


/* Initialization Functions --------------------------------------------------*/

/** 
 * @brief  Initializes all the fields in the given ACS struct
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
*/
void initializeACS(ACSType* acs);

#endif
