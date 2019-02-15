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


/* Public functions prototypes ---------------------------------------------*/
void FindSolarVector(double xp, double xm, double yp, double ym, double zp, double zm, double* result_vector);