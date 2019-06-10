/**
  ******************************************************************************
  * @file           FeedbackControl.h
  * @brief          Contains implementations of the feedback control states in the ACS
  ******************************************************************************
  ** Controller implementations are in OrientationController.c and
  * StabilizationController.c. A call of findErrorVectors() is required before 
  * either runOrientationController() or runStabilizationController() for the 
  * ACS to work correctly.
  * 
  * Created by Galen Savidge. Edited 5/13/2019.
  ******************************************************************************
  */

#ifndef FEEDBACKCONTROL_H
#define FEEDBACKCONTROL_H

/* Includes ------------------------------------------------------------------*/
#include <ACS.h>

#define TR_V_RAIL 3.3f // Volts
#define TR_MAX_V 1.0f // Volts
#define TR_MAXDIP 0.5 /**< Torque rod max dipole moment (A*m^2) */


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
void runBdotController(ACSType* acs);

 /**
 * @brief  Runs the feedback controller used to move the craft over large angles
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  first_step: 1 right after state transitions, 0 otherwise
 * @return Updates acs->rw_PWM
 */
void runOrientationController(ACSType* acs, int first_step);

/**
 * @brief  Runs the feedback controller used to stabilize the craft when error is low
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  err: generally either acs.z_err or acs.err
 * @param  init_error: 1 right after state transitions, 0 otherwise
 * @param  reset_integrator: 1 after this function has not been called for a while, 0 otherwise
 * @return Updates acs->rw_PWM
 */
void runStabilizationController(ACSType* acs, Matrix err, int init_error, int reset_integrator);

/**
 * @brief  Finds reaction wheel PWM to achieve a desired craft wdot
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  wdot_desired: desired craft wdot
 * @param  rw_pwm: allocated Matrix to hold the reaction wheel PWM
 * @param  dt: expected time until next feedback controller update
 * @return Updates acs->rw_PWM
 */
void wdot2rw_pwm(ACSType* acs, Matrix wdot_desired);

#endif /* FEEDBACKCONTROL_H */
