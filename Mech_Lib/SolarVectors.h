/**
  ******************************************************************************
  * @file           : SolarVectors.h
  * @brief          : Header for the SolarVectors module.
  ******************************************************************************
  ** This module contains functions for finding and printing solar vectors from
	* solar panel current measurements.
	*
	* Created by Galen Savidge 2/15/2019.
  ******************************************************************************
  */
 
/* Includes ------------------------------------------------------------------*/
#include <Matrix.h>

/* Public functions prototypes ---------------------------------------------*/

/** 
 * @brief  Calculates a solar vector from solar panel current measurements
 * @param  Six current measurements, one for each solar panel
 * @return A normalized 3x1 column vector Matrix
*/
Matrix findSolarVector(double xp, double xm, double yp, double ym, double zp, double zm);

/** 
 * @brief  Prints yaw and pitch of a solar vector relative to the +X unit vector
 * @param  Normalized 3x1 column vector Matrix, string to hold the result
 * @return None
*/
void printSolarVector(Matrix v, char* string);