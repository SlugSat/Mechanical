/**
  ******************************************************************************
  * @file           : AttitudeEstimation.h
  * @brief          : Header file for the Attitude Control System (ACS).
  ******************************************************************************
  ** 
	* 
	* Created by Galen Savidge. Edited 3/6/2019.
  ******************************************************************************
  */

#ifndef ACS_H
#define ACS_H

/* Includes ------------------------------------------------------------------*/
// Libraries
#include <Matrix.h>
#include <DigitalFilters.h>
#include <main.h>

// ACS modules
#include <SolarVectors.h>


/* Constants -----------------------------------------------------------------*/
#define NUM_SOLAR_PANELS 6

 
/* Datatypes -----------------------------------------------------------------*/
typedef struct {
	// Craft DCM
	Matrix R;
	
	// Vectors
	Matrix gyro_vector;
	Matrix gyro_bias;
	Matrix mag_vector;
	Matrix solar_vector;
	Matrix sv_inertial;
	Matrix mag_inertial;
	
	// Sensors
	I2C_HandleTypeDef* hi2c;
	MovingAvgFilter mag_filter;
	MovingAvgFilter sv_filter;
	uint32_t sv_raw[NUM_SOLAR_PANELS]; // For DMA to use
	
	// Solar panel status
	SV_Status sun_status;
}ACSType;
 
#endif
