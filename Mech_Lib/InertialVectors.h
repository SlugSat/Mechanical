/**
  ******************************************************************************
  * @file           InertialVectors.h
  * @brief          Contains functions to find sun and magnetic field inertial vectors
  ******************************************************************************
	* Created by Galen Savidge. Edited 5/12/2019.
  ******************************************************************************
  */

#ifndef INERTIALVECTORS_H
#define INERTIALVECTORS_H

/* Includes ------------------------------------------------------------------*/
#include <ACS.h>


/* Public Functions ----------------------------------------------------------*/

/**
 * @brief  Finds the Earth to sun inertial vector using the current Julian date
 * @param  acs: ACSType with updated julian_date
 * @return Updates acs->sv_inertial
 */
void findSunInertial(ACSType* acs);


/**
 * @brief  Uses IGRF to find the Earth's magnetic field at the craft's position
 * @param  acs: ACSType with updated: julian_date, longitude, latitude, altitude, craft_j2000
 * @return Updates acs->mag_inertial
 */
void findMagInertial(ACSType* acs);

#endif /* INERTIALVECTORS_H */
