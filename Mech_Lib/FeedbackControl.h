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
 * @brief  Uses the craft orientation and inertial vectors to find error
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return Updates z_err, n_err, and err in acs
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
 * @brief  Runs the feedback controller used to move the craft over large angles
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  dt: time in seconds since last call of this function
 * @param  first_step: 1 right after state transitions, 0 otherwise
 * @return Updates acs->rw_PWM
 */
void runOrientationController(ACSType* acs, int first_step);

/**
 * @brief  Runs the feedback controller used to stabilize the craft when error is low
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  dt: time in seconds since last call of this function
 * @param  first_step: 1 right after state transitions, 0 otherwise
 * @return Updates acs->rw_PWM
 */
void runStabilizationController(ACSType* acs, int first_step);

/**
 * @brief  Finds reaction wheel PWM to achieve a desired craft wdot
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  wdot_desired: desired craft wdot
 * @param  rw_pwm: allocated Matrix to hold the reaction wheel PWM
 * @param  dt: expected time until next feedback controller update
 * @return Updates acs->rw_PWM
 */
void wdot2rw_pwm(ACSType* acs, Matrix wdot_desired, float dt);

#endif /* FEEDBACKCONTROL_H */
