/**
  ******************************************************************************
  * @file           : InertialVectors.h
  * @brief          : Contains functions to find sun and magnetic field inertial vectors
  ******************************************************************************
	* Created by Galen Savidge. Edited 5/4/2019.
  ******************************************************************************
  */

#ifndef INERTIALVECTORS_H
#define INERTIALVECTORS_H

/* Includes ------------------------------------------------------------------*/
#include <ACS.h>


/* Public Functions ----------------------------------------------------------*/

/**
 * @brief  Finds the Earth to sun inertial vector using the current Julian date
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return Updates acs->sv_inertial
 */
void findSunInertial(ACSType* acs);


/**
 * @brief  
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return Updates acs->sv_inertial
 */
void findMagInertial(ACSType* acs);

#endif /* INERTIALVECTORS_H */
