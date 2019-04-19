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

void findErrorVectors(ACSType* acs);

/**
 * @brief  Description
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
 */
void runBdotController(ACSType* acs);

 /**
 * @brief  Description
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
 */
void runOrientationController(ACSType* acs);

/**
 * @brief  Description
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
 */
void runStabilizationController(ACSType* acs);
 
#endif
