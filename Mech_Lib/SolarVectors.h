/**
  ******************************************************************************
  * @file           : SolarVectors.h
  * @brief          : Header for the SolarVectors module.
  ******************************************************************************
  ** This module contains functions for finding and printing solar vectors from
	* solar panel current measurements.
	*
	* Created by Galen Savidge. Edited 2/16/2019.
  ******************************************************************************
  */
 
 #ifndef SOLARVECTOR_H
 #define SOLARVECTOR_H
 
/* Includes ------------------------------------------------------------------*/
#include <Matrix.h>

/* Public functions prototypes ---------------------------------------------*/

/** 
 * @brief  Calculates a solar vector from solar panel current measurements
 * @param  Six current measurements, one for each solar panel
 * @param  v: an allocated 3x1 column vector Matrix to hold the solar vector
 * @return A normalized 3x1 column vector Matrix
*/
void findSolarVector(double xp, double xm, double yp, double ym, double zp, double zm, Matrix v);

/** 
 * @brief  Prints yaw and pitch of a solar vector relative to the +X unit vector
 * @param  Normalized 3x1 column vector Matrix, string to hold the result
 * @return None
*/
void printSolarVector(Matrix v, char* string);

#endif /* SOLARVECTOR_H */
