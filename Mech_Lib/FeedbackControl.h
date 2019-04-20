/**
  ******************************************************************************
  * @file           : FeedbackControl.h
  * @brief          : Contains implementations for the feedback control states in the ACS
  ******************************************************************************
  ** 
	* As of now these functions change actuator outputs inside of the function
	* itself but that is subject to change.
	* 
	* Created by Galen Savidge. Edited 4/18/2019.
  ******************************************************************************
  */

#ifndef FEEDBACKCONTROL_H
#define FEEDBACKCONTROL_H

/* Includes ------------------------------------------------------------------*/
#include <ACS.h>


/* Public Functions ----------------------------------------------------------*/

/**
 * @brief  Description
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
 */
void findErrorVectors(ACSType* acs);

/**
 * @brief  Description
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  dt: time in seconds since last call of this function
 * @return None
 */
void runBdotController(ACSType* acs, float dt);

 /**
 * @brief  Description
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  dt: time in seconds since last call of this function
 * @return None
 */
void runOrientationController(ACSType* acs, float dt);

/**
 * @brief  Description
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  dt: time in seconds since last call of this function
 * @return None
 */
void runStabilizationController(ACSType* acs, float dt);

/**
 * @brief  Function will update the acs->sv_inertial with current julian date
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  julianDate: current julian date
 * @return None
 */
 void findSunInertial(ACSType* acs, double julianDate);
 
#endif
