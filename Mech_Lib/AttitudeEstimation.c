/**
  ******************************************************************************
  * @file           : AttitudeEstimation.h
  * @brief          : Source file for the AttitudEstimation module.
  ******************************************************************************
  ** This module contains the code to run closed loop integration of the gyro
	* using feedback from the magnetometer and solar vector.
	* 
	* Created by Galen Savidge. Edited 2/23/2019.
  ******************************************************************************
  */

#include <AttitudeEstimation.h>

/** 
 * @brief  Allocates and initializes a 3x3 DCM Matrix
 * @param  Euler angles for the initial DCM
 * @return The new DCM Matrix
*/
Matrix initializeDCM(double yaw, double pitch, double roll) {
	Matrix r = newMatrix(3, 3);
	matrixSet(r, 1, 1, 1.0);
	matrixSet(r, 2, 2, 1.0);
	matrixSet(r, 3, 3, 1.0);
	return r;
}
