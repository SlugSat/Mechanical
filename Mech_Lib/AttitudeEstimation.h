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
 * @param  Euler angles for the initial DCM
 * @return The new DCM Matrix
*/
Matrix initializeDCM(float yaw, float pitch, float roll);


/* Sensor Fusion Functions --------------------------------------------------*/

/** 
 * @brief  Updates the craft attitude estimate using sensor data in acs
 * @param  acs: an initialized ACS struct which has up-to-date sensor data
 * @param  dt: time in seconds since last call of this function
 * @return None
*/
void updateAttitudeEstimate(ACSType* acs);

/** 
 * @brief  Performs closed loop integration on the given DCM using the Rexp form
 * @param  R: the DCM (initialize to I3 before first use)
 * @param  yaw_pitch_roll: pointer to float[3] which hold yaw (0) pitch (1) and roll (2) after the function returns
 * @return None
*/
void findEulerAngles(Matrix R, float* yaw_pitch_roll);

#endif /* ATTITUDEESTIMATION_H */
