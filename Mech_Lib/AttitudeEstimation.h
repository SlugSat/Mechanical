/**
  ******************************************************************************
  * @file           AttitudeEstimation.h
  * @brief          Sensor fusion and attitude determination
  ******************************************************************************
  ** This module contains the code to run closed loop gyro integration using
  * feedback from the magnetometer and solar vector. Call initalizeACS() in 
  * ACS.h to initialize an Attitude Control System struct, then pass it into
  * updateAttitudeEstimate() every loop to get the craft's Direction Consine
  * Matrix (DCM).
  * 
  * Created by Galen Savidge. Edited 5/12/2019.
  ******************************************************************************
  */

#ifndef ATTITUDEESTIMATION_H
#define ATTITUDEESTIMATION_H
 
/* Includes ------------------------------------------------------------------*/
#include "ACS.h"


/* Initialization Functions --------------------------------------------------*/

/** 
 * @brief  Allocates and initializes a 3x3 DCM Matrix
 * @param  Initial Euler angles for the DCM
 * @return The new DCM Matrix
*/
Matrix initializeDCM(float yaw, float pitch, float roll);


/* Sensor Fusion Functions --------------------------------------------------*/

/** 
 * @brief  Updates the craft attitude estimate using sensor data in acs
 * @param  acs: ACSType with updated: mag_vector, solar_vector, gyro_vector, mag_inertial, sv_inertial, dt
 * @return Updates: R, Rt, gyro_bias, gyro_bias_dot_norm
*/
void updateAttitudeEstimate(ACSType* acs);

/** 
 * @brief  Finds intrinsic zyx Euler angles in inertial frame from a body->inertial DCM
 * @param  R: the DCM
 * @param  yaw_pitch_roll: pointer to float[3] which hold yaw (0) pitch (1) and roll (2) after the function returns
 * @return Updates contents of yaw_pitch_roll
*/
void findEulerAngles(Matrix R, float* yaw_pitch_roll);

#endif /* ATTITUDEESTIMATION_H */
