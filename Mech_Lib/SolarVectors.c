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

/** 
 * @brief  Calculates a solar vector from solar panel current measurements
 * @param  Six current measurements, one for each solar panel
 * @param  v: an allocated 3x1 column vector Matrix to hold the solar vector
 * @return A normalized 3x1 column vector Matrix
*/
void findSolarVector(double xp, double xm, double yp, double ym, double zp, double zm, Matrix v) {
	// Calculate magnitude of solar vector
	double vector_mag = sqrt(pow(xp - xm, 2) + pow(yp - ym, 2) + pow(zp - zm, 2));
	
	// Set x component
	matrixSet(v, 1, 1, (xp - xm)/vector_mag);
	
	// Set y component
	matrixSet(v, 2, 1, (yp - ym)/vector_mag);
	
	// Set z component
	matrixSet(v, 3, 1, (zp - zm)/vector_mag);
}
	
/** 
 * @brief  Prints yaw and pitch (in degrees) of a solar vector relative to the +X unit vector
 * @param  Normalized 3x1 column vector Matrix, string to hold the result
 * @return None
*/
void printSolarVector(Matrix v, char* string) {
	double vector_x = matrixGetElement(v, 1, 1);
	double vector_y = matrixGetElement(v, 2, 1);
	double vector_z = matrixGetElement(v, 3, 1);
	
	// Convert vector into yaw/pitch angles referenced to the +X unit vector (1, 0, 0)
	double yaw_degrees = 180*acos(vector_x/sqrt(pow(vector_x, 2) + pow(vector_y, 2)))/PI;
	if(vector_y < 0) {
		yaw_degrees *= -1;
	}
	
	double pitch_degees = 180*acos(sqrt(pow(vector_x, 2) + pow(vector_y, 2)))/PI;
	if(vector_z < 0) {
		pitch_degees *= -1;
	}
	
	sprintf(string, "yaw: %3.1f pitch: %3.1f\r\n", yaw_degrees, pitch_degees);
}
