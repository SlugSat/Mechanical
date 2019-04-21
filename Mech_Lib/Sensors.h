/**
  ******************************************************************************
  * @file           : Sensors.h
  * @brief          : Header for the Sensors module.
  ******************************************************************************
  ** This module contains functions to read from the physical sensors on the 
    * satellite.
	* 
	* Created by Galen Savidge. Edited 4/21/2019.
  ******************************************************************************
  */

#ifndef SENSORS_H
#define SENSORS_H
 
/* Includes ------------------------------------------------------------------*/
#include "ACS.h"


/* Public Functions ----------------------------------------------------------*/

/** 
 * @brief  Initializes all the fields in the given ACS struct
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
*/
void initializeSensors(ACSType* acs, I2C_HandleTypeDef* hi2c, ADC_HandleTypeDef* hadc);


/** 
 * @brief  Reads sensors and updates gyro_vector, mag_vector, and solar_vector in acs
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  hi2c: i2c handle for the IMU
 * @return None
*/
void readSensors(ACSType* acs, I2C_HandleTypeDef* hi2c);


#endif /* SENSORS_H */
