/*
Author: Gabriel Barbosa

Reference: All functions are pulled from geomag70.c found here:
	https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html
	
Date: 4/29/2019
*/

#include <Matrix.h>

/**
 * @brief function that returns the mag field vector
 * @param 
 *		JD 			- Julian Date
 *		longitude 	- in degrees
 *		latitude 	- in degrees
 *		altitude 	- in km
 * @return
 *		mag_NED		- Mag field vector with North, East, Down components in nT 
 */
void get_mag_inertial(double JD, float longitude, float latitude, float altitude, Matrix mag_NED);
