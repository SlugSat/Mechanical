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


// THESE NEED CHANGING POST-CALIBRATION
#define SUN_MIN_THRESH 0.1 // Threshold (volts) for a panel to be considered in the sun at all
#define VECTOR_MAG_THRESH_LOW 0.1 // Hysteresis thresholds (volts) for the vector magnitude to determine if the satellite is in the sun
#define VECTOR_MAG_THRESH_HIGH 0.15

/** 
 * @brief  Calculates a solar vector from solar panel current measurements
 * @param  adc_readings: [xp, xm, yp, ym, zp, (zm)]
 * @param  num_panels: the number of solar panels to use (6 or 5)
 * @param  v: an allocated 3x1 column vector Matrix to hold the solar vector
 * @return SV_FOUND if a valid solar vector was found, SV_NOTFOUND otherwise
*/
SV_Status findSolarVector(float* adc_readings, char num_panels, Matrix v) {
	static SV_Status last_dark = SV_DARK;
	static SV_Status last_valid = SV_NOTFOUND;
	SV_Status new_status;
	
	float xp, xm, yp, ym, zp, zm;
	
	xp = adc_readings[0]*XP_SCALE;
	xm = adc_readings[1]*XN_SCALE;
	yp = adc_readings[2]*YP_SCALE;
	ym = adc_readings[3]*YN_SCALE;
	zp = adc_readings[4]*ZP_SCALE;
	
	if(num_panels == 6) {
		zm = adc_readings[5]*ZN_SCALE;
	}
	else if(num_panels == 5) {
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
	
	// Detect indeterminate solar vectors
	if(num_panels == 5 && zp < SUN_MIN_THRESH) {
			last_valid = SV_NOTFOUND;
	}
	else {
			last_valid = SV_FOUND;
		}
	
	// Dark detection
	if(vector_mag < VECTOR_MAG_THRESH_LOW && last_dark == SV_FOUND) {
		last_dark = SV_DARK;
	}
	else if(vector_mag > VECTOR_MAG_THRESH_HIGH && last_dark == SV_DARK) {
		last_dark = SV_FOUND;
	}
	
	if(last_dark == SV_FOUND) {
		new_status = last_valid;
	}
	else {
		new_status = last_dark;
	}
	
	return new_status;
}
	
/** 
 * @brief  Prints yaw and pitch (in degrees, from inertial frame) of a solar vector relative to the +X unit vector
 * @param  Normalized 3x1 column vector Matrix, string to hold the result
 * @return None
*/
void printSolarVector(Matrix v, char* string) {
	static uint8_t init_run = 0;
	static Matrix v_rcross;
	static Matrix xB;
	static Matrix yB;
	static Matrix n;
	static Matrix xhat_proj;
	
	if(init_run == 0) {
		v_rcross = newMatrix(3, 3);
		xB = make3x1Vector(1, 0, 0);
		yB = make3x1Vector(0, 1, 0);
		n = newMatrix(3, 1);
		xhat_proj = newMatrix(3, 1);
		
		init_run = 1;
	}
	
	// Find normal vector to inertial xy plane
	vectorRcross(v, v_rcross);
	matrixMult(v_rcross, yB, n);
	
	float n_norm = vectorNorm(n);
	if(n_norm != 0) {
		matrixScale(n, 1.0/n_norm); // Normalize n
	}
	
	// Find normalized projection of xB onto inertial xy plane
	matrixScale(n, matrixGetElement(n, 1, 1));
	matrixSubtract(xB, n, xhat_proj);
	float xhat_proj_norm = vectorNorm(xhat_proj);
	if(xhat_proj_norm != 0) {
		matrixScale(xhat_proj, 1.0/xhat_proj_norm);
	}
	
	// Pitch = acos(xhat_proj_x)
	float pitch_degrees = 180*acos(matrixGetElement(xhat_proj, 1, 1))/PI;
	
	// Yaw = acos(xhat_proj*v)
	float yaw_degrees = 180*acos(vectorDotProduct(xhat_proj, v))/PI;
	
	float vector_x = matrixGetElement(v, 1, 1);
	float vector_y = matrixGetElement(v, 2, 1);
	float vector_z = matrixGetElement(v, 3, 1);
	
	// Convert vector into yaw/pitch angles referenced to the +X unit vector (1, 0, 0)
	if(vector_y < 0) {
		yaw_degrees *= -1;
	}
	
	if(vector_z < 0) {
		pitch_degrees *= -1;
	}
	
	sprintf(string, "yaw: %3.1f pitch: %3.1f\r\n", yaw_degrees, pitch_degrees);
}
