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
#include <ACS.h>


#define EARTH_RADIUS_KM 6371.2
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
 * @brief  
 * @param  v_j2000: a column vector Matrix in the J2000 frame
 * @param  
 * @return 
*/
void J2000_2_LongLatAlt(ACSType* acs, Matrix v_j2000, float* lng, float* lat, float* alt);

/** 
 * @brief  
 * @param  v_ned: a column vector Matrix in North/East/Down coordinates
 * @param  
 * @return 
*/
void NED_2_J2000(ACSType* acs, Matrix v_ned, Matrix v_j2000);

#endif /* REFERENCEFRAMES_H */
