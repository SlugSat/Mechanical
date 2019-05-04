/**
  ******************************************************************************
  * @file           : ReferenceFrames.h
  * @brief          : Contains functions to translate between reference frames.
  ******************************************************************************
  ** 
	* 
	* Created by Galen Savidge. Edited 5/2/2019.
  ******************************************************************************
  */

#ifndef REFERENCEFRAMES_H
#define REFERENCEFRAMES_H

/* Includes ------------------------------------------------------------------*/
#include <Matrix.h>


#define PI 3.14159265359
#define EARTH_RADIUS_KM 6378
#define RAD2DEG (180.0/PI)


/* Public Functions ----------------------------------------------------------*/
/** 
 * @brief  
 * @param  Euler angles
 * @return R contains the DCM
*/
void makeDCM_zyx(Matrix R, float yaw, float pitch, float roll);

/** 
 * @brief  Converts vectors in the J2000 frame to the ecliptic ECI frame
 * @param  v_j2000: a column vector Matrix in the J2000 frame
 * @param  v_ecliptic: an allocated column vector Matrix to hold the result
 * @return v_ecliptic now holds v_j2000 converted into the ecliptic frame
*/
void J2000_2_ecliptic(Matrix v_j2000, Matrix v_ecliptic);

/** 
 * @brief  Converts J2000 vectors to longitude, latitude, and altitude on Earth
 * @param  v_j2000: a column vector Matrix in the J2000 frame
 * @param  JD: the current Julian date (days, with fraction)
 * @param  lng, lat, alt: variables to return results
 * @return longitude & latitude (degrees), altitude (meters)
*/
void J2000_2_LongLatAlt(Matrix v_j2000, double JD, float* lng, float* lat, float* alt);

/** 
 * @brief  Converts body centered vectors in a North/East/Down (NED) frame to J2000
 * @param  v_ned: a column vector Matrix in NED
 * @param  c_j2000: craft position in J2000
 * @param  JD: the current Julian date (days, with fraction)
 * @param  v_j2000: an allocated column vector Matrix
 * @return v_j2000 now holds v_ned in J2000
*/
void NED_2_J2000(Matrix v_ned, Matrix c_j2000, double JD, Matrix v_j2000);

#endif /* REFERENCEFRAMES_H */
