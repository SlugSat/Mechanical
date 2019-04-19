/**
  ******************************************************************************
  * @file           : AttitudeEstimation.h
  * @brief          : Header for the AttitudEstimation module.
  ******************************************************************************
  ** This module contains the code to run closed loop integration of the gyro
	* using feedback from the magnetometer and solar vector.
	* 
	* Created by Galen Savidge. Edited 3/6/2019.
  ******************************************************************************
  */

 #ifndef ATTITUDEESTIMATION_H
 #define ATTITUDEESTIMATION_H
 
/* Includes ------------------------------------------------------------------*/
#include <Matrix.h>
#include <DigitalFilters.h>
#include <SolarVectors.h>
#include <main.h>

/* Constants -----------------------------------------------------------------*/
#define NUM_SOLAR_PANELS 6
#define SENSOR_READ_DELAY_MS 50

/* Datatypes -----------------------------------------------------------------*/
typedef struct {
	// Craft DCM
	Matrix R;
	
	// Vectors
	Matrix gyro_vector;
	Matrix gyro_bias;
	Matrix mag_vector;
	Matrix solar_vector;
	Matrix sv_inertial;
	Matrix mag_inertial;
	
	// Sensors
	I2C_HandleTypeDef* hi2c;
	MovingAvgFilter mag_filter;
	MovingAvgFilter sv_filter;
	uint32_t sv_raw[NUM_SOLAR_PANELS]; // For DMA to use
	
	// Solar panel status
	SV_Status sun_status;
}ACSType;


/* Initialization Functions ---------------------------------------------*/

/** 
 * @brief  Initializes all the fields in the given ACS struct
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
*/
void initializeACS(ACSType* acs);

/** 
 * @brief  Initializes all the fields in the given ACS struct
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
*/
void initializeSensors(ACSType* acs, I2C_HandleTypeDef* hi2c, ADC_HandleTypeDef* hadc);

/** 
 * @brief  Allocates and initializes a 3x3 DCM Matrix
 * @param  Euler angles for the initial DCM
 * @return The new DCM Matrix
*/
Matrix initializeDCM(float yaw, float pitch, float roll);


/* Sensor Functions ----------------------------------------------------*/

/** 
 * @brief  Reads sensors and updates gyro_vector, mag_vector, and solar_vector in acs
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
*/
void readSensors(ACSType* acs);

/** 
 * @brief  Reads sensor data from serial and updates acs
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
*/
void readSensorsFromSerial(ACSType* acs, UART_HandleTypeDef* huart); // To-do


/* Sensor Fusion Functions -------------------------------------------*/

/** 
 * @brief  Updates the craft attitude estimate using sensor data in acs
 * @param  acs: an initialized ACS struct which has up-to-date sensor data
 * @param  dt: time in seconds since last call of this function
 * @return None
*/
void updateAttitudeEstimate(ACSType* acs, float dt);

/** 
 * @brief  Performs closed loop integration on the given DCM using the Rexp form
 * @param  acs: an initialized ACS struct which has up-to-date sensor data
 * @param  K*: feedback constants for the mag and solar vectors
 * @param  dt: time in seconds since last call of this function
 * @return None
*/
void integrateDCM(ACSType* acs, float Kp_mag, float Ki_mag, float Kp_sv, float Ki_sv, float dt);

/** 
 * @brief  Performs closed loop integration on the given DCM using the Rexp form
 * @param  R: the DCM (initialize to I3 before first use)
 * @param  yaw_pitch_roll: pointer to float[3] which hold yaw (0) pitch (1) and roll (2) after the function returns
 * @return None
*/
void findEulerAngles(Matrix R, float* yaw_pitch_roll);

#endif /* ATTITUDEESTIMATION_H */
