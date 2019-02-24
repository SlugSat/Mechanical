/**
  ******************************************************************************
  * @file           : AttitudeEstimation.h
  * @brief          : Header for the AttitudEstimation module.
  ******************************************************************************
  ** This module contains the code to run closed loop integration of the gyro
	* using feedback from the magnetometer and solar vector.
	* 
	* Created by Galen Savidge. Edited 2/23/2019.
  ******************************************************************************
  */

 #ifndef ATTITUDEESTIMATION_H
 #define ATTITUDEESTIMATION_H
 
/* Includes ------------------------------------------------------------------*/
#include <Matrix.h>
#include <SolarVectors.h>
#include <BNO055_IMU.h>

/* Public functions prototypes ---------------------------------------------*/

/** 
 * @brief  Allocates and initializes a 3x3 DCM Matrix (currently always returns I3)
 * @param  Euler angles for the initial DCM
 * @return The new DCM Matrix
*/
Matrix initializeDCM(double yaw, double pitch, double roll);

/** 
 * @brief  Performs closed loop integration on the given DCM using the Rexp form
 * @param  R: the DCM (initialize to I3 before first use)
 * @param  gyro: 
 * @param  mag: 
 * @param  sv: the solar vector from findSolarVector() in the SolarVectors module
 * @param  newR: 
 * @return None
*/
void integrateDCM(Matrix R, Matrix gyro, Matrix mag, Matrix sv, double dt, Matrix newR);

#endif /* ATTITUDEESTIMATION_H */
