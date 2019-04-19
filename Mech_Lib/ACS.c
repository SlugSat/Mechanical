/**
  ******************************************************************************
  * @file           : AttitudeEstimation.h
  * @brief          : Header file for the Attitude Control System (ACS).
  ******************************************************************************
  ** 
	* 
	* Created by Galen Savidge. Edited 4/19/2019.
  ******************************************************************************
  */

// LIBRARIES
#include <ACS.h>
#include <BNO055_IMU.h>
#include <AttitudeEstimation.h>
#include <math.h>


// PUBLIC FUNCTIONS

void initializeACS(ACSType* acs) {
	// Allocate Matrix structs
	acs->R = initializeDCM(0, 0, 0);
	acs->gyro_vector = newMatrix(3, 1);
	acs->gyro_bias = newMatrix(3, 1);
	acs->mag_vector = newMatrix(3, 1);
	acs->solar_vector = newMatrix(3, 1);
	acs->sv_inertial = newMatrix(3, 1);
	acs->mag_inertial = newMatrix(3, 1);
}
