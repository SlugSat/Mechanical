/**
  ******************************************************************************
  * @file           : SolarVectors.c
  * @brief          : Source file for the SolarVectors module.
  ******************************************************************************
  ** This module contains functions for finding and printing solar vectors from
	* solar panel current measurements.
	*
	* Created by Galen Savidge. Edited 2/16/2019.
  ******************************************************************************
  */
	
#include "SolarVectors.h"
#include <stdio.h>
#include <math.h>

#define PI 3.1416

#define XP_SCALE 1.0 // Should always be 1
#define XN_SCALE 0.7679
#define YP_SCALE 0.6143
#define YN_SCALE 0.8431
#define ZP_SCALE 0.6418
#define ZN_SCALE 0.9348

#define SUN_MIN_THRESH 128 // 12-bit ADC value; NEEDS CHANGING POST CALIBRATION

/** 
 * @brief  Calculates a solar vector from solar panel current measurements
 * @param  adc_readings: [xp, xm, yp, ym, zp, (zm)]
 * @param  num_panels: the number of solar panels to use (6 or 5)
 * @param  v: an allocated 3x1 column vector Matrix to hold the solar vector
 * @return SV_FOUND if a valid solar vector was found, SV_NOTFOUND otherwise
*/
SV_Status findSolarVector(uint32_t* adc_readings, char num_panels, Matrix v) {
	float xp, xm, yp, ym, zp, zm;
	if(num_panels == 6) {
		xp = adc_readings[0]*XP_SCALE;
		xm = adc_readings[1]*XN_SCALE;
		yp = adc_readings[2]*YP_SCALE;
		ym = adc_readings[3]*YN_SCALE;
		zp = adc_readings[4]*ZP_SCALE;
		zm = adc_readings[5]*ZN_SCALE;
	}
	else if(num_panels == 5) {
		xp = adc_readings[0]*XP_SCALE;
		xm = adc_readings[1]*XN_SCALE;
		yp = adc_readings[2]*YP_SCALE;
		ym = adc_readings[3]*YN_SCALE;
		zp = adc_readings[4]*ZP_SCALE;
		zm = 0;
	}
	
	// Calculate magnitude of solar vector
	float vector_mag = sqrt(pow(xp - xm, 2) + pow(yp - ym, 2) + pow(zp - zm, 2));
	
	// Set x component
	matrixSet(v, 1, 1, (xp - xm)/vector_mag);
	
	// Set y component
	matrixSet(v, 2, 1, (yp - ym)/vector_mag);
	
	// Set z component
	matrixSet(v, 3, 1, (zp - zm)/vector_mag);
	
	if(xp > SUN_MIN_THRESH || xm > SUN_MIN_THRESH || yp > SUN_MIN_THRESH || ym > SUN_MIN_THRESH || zp > SUN_MIN_THRESH || zm > SUN_MIN_THRESH) {
		return SV_FOUND;
	}
	return SV_NOTFOUND;
}
	
/** 
 * @brief  Prints yaw and pitch (in degrees) of a solar vector relative to the +X unit vector
 * @param  Normalized 3x1 column vector Matrix, string to hold the result
 * @return None
*/
void printSolarVector(Matrix v, char* string) {
	static uint8_t init_run = 0;
	static Matrix xhat_proj;
	
	if(init_run == 0) {
		xhat_proj = newMatrix(3, 1);
		init_run = 1;
	}
	
	float vector_x = matrixGetElement(v, 1, 1);
	float vector_y = matrixGetElement(v, 2, 1);
	float vector_z = matrixGetElement(v, 3, 1);
	
	// Convert vector into yaw/pitch angles referenced to the +X unit vector (1, 0, 0)
	float yaw_degrees = 180*acos(vector_x/sqrt(pow(vector_x, 2) + pow(vector_y, 2)))/PI;
	if(vector_y < 0) {
		yaw_degrees *= -1;
	}
	
	float pitch_degees = 180*acos(sqrt(pow(vector_x, 2) + pow(vector_y, 2)))/PI;
	if(vector_z < 0) {
		pitch_degees *= -1;
	}
	
	sprintf(string, "yaw: %3.1f pitch: %3.1f\r\n", yaw_degrees, pitch_degees);
}
