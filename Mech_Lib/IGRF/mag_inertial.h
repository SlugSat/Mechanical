/*
Author: Gabriel Barbosa

Reference: All functions are pulled from geomag70.c found here:
	https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html
	
Date: 4/29/2019
*/

#include <Matrix.h>

/**
 * @brief 
 * @param
 * @return
 */
void get_mag_inertial(double JD, float longitude, float latitude, float altitude, Matrix mag_NED);
