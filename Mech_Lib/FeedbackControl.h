/**
  ******************************************************************************
  * @file           : FeedbackControl.h
  * @brief          : Contains implementations for the feedback control states in the ACS
  ******************************************************************************
  * 
	* Controller implementations are in OrientationController.c and
	* StabilizationController.c.
	* 
	* Created by Galen Savidge. Edited 4/21/2019.
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
 * @param  first_step: 1 right after state transitions, 0 otherwise
 * @return None
 */
void runOrientationController(ACSType* acs, float dt, int first_step);

/**
 * @brief  Description
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  dt: time in seconds since last call of this function
 * @param  first_step: 1 right after state transitions, 0 otherwise
 * @return None
 */
void runStabilizationController(ACSType* acs, float dt, int first_step);

/**
 * @brief  Function will update the acs->sv_inertial with current julian date
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  julianDate: current julian date
 * @return None
 */
void findSunInertial(ACSType* acs, double julianDate);

/**
 * @brief  Finds reaction wheel PWM to achieve a desired craft wdot
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  wdot_desired: desired craft wdot
 * @param  rw_pwm: allocated Matrix to hold the reaction wheel PWM
 * @param  dt: expected time until next feedback controller update
 * @return None
 */
void wdot2rw_pwm(ACSType* acs, Matrix wdot_desired, float dt);

#endif /* FEEDBACKCONTROL_H */
