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
 
/* Includes -----------------------------------------------------------------*/
#include <Matrix.h>
#include <stdint.h>

/* Public defines -----------------------------------------------------------*/

#define XP_SCALE 1.0 // Should always be 1
#define XN_SCALE 0.81669394435
#define YP_SCALE 0.51860928166
#define YN_SCALE 0.7354584972
#define ZP_SCALE 0.74533233756
#define ZN_SCALE 0.94866920152

/* Public typedefs ----------------------------------------------------------*/

typedef enum {
	SV_FOUND,
	SV_NOTFOUND
}SV_Status;

/* Public functions prototypes ----------------------------------------------*/

/** 
 * @brief  Calculates a solar vector from solar panel current measurements
 * @param  adc_readings: filtered readings from the ADC
 * @param  num_panels: the number of solar panels to use (6 or 5)
 * @param  v: an allocated 3x1 column vector Matrix to hold the solar vector
 * @return SV_FOUND if a valid solar vector was found, SV_NOTFOUND otherwise
*/
SV_Status findSolarVector(float* adc_readings, char num_panels, Matrix v);

/** 
 * @brief  Prints yaw and pitch of a solar vector relative to the +X unit vector
 * @param  Normalized 3x1 column vector Matrix, string to hold the result
 * @return None
*/
void printSolarVector(Matrix v, char* string);

#endif /* SOLARVECTOR_H */
