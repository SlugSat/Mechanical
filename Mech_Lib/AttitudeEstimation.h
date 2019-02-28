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
 * @param  bias: estimated bias of the gyro (updated automatically by the function)
 * @param  gyro: gyro reading from IMU in rad/s
 * @param  mag: magnetometer reading from the IMU
 * @param  sv: the solar vector from findSolarVector() in the SolarVectors module
 * @param  mag_inertial: magnetometer reading we expect in the inertial frame
 * @param  sv_inertial: solar vector we expect in the inertial frame
 * @param  K*: feedback constants for the mag and solar vectors
 * @param  dt: time since last call of integrateDCM() in seconds
 * @return None
*/
void integrateDCM(Matrix R, Matrix bias, Matrix gyro, Matrix mag, Matrix sv, 
		Matrix mag_inertial, Matrix sv_inertial, double Kp_mag, double Ki_mag,
		double Kp_sv, double Ki_sv, double dt);

/** 
 * @brief  Performs closed loop integration on the given DCM using the Rexp form
 * @param  R: the DCM (initialize to I3 before first use)
 * @param  yaw_pitch_roll: pointer to float[3] which hold yaw (0) pitch (1) and roll (2) after the function returns
 * @return None
*/
void findEulerAngles(Matrix R, float* yaw_pitch_roll);

#endif /* ATTITUDEESTIMATION_H */
