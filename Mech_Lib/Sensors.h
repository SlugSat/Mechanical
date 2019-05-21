/**
  ******************************************************************************
  * @file           Sensors.h
  * @brief          Contains functions to read from the physical sensors on the satellite
  ******************************************************************************
  * Created by Galen Savidge. Edited 5/12/2019.
  ******************************************************************************
  */

#ifndef SENSORS_H
#define SENSORS_H
 
/* Includes ------------------------------------------------------------------*/
#include "ACS.h"


/* Public Functions ----------------------------------------------------------*/

/** 
 * @brief  Initializes sensor hardware and the digital filters in an ACS struct
 *
 * Initializes the IMU and starts the ADCs. Also initializes each 
 * MovingAverageFilter in acs and runs them until they are each full of readings.
 * 
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
*/
void initializeSensors(ACSType* acs, SPI_HandleTypeDef* hspi, ADC_HandleTypeDef* hadc);


/** 
 * @brief  Reads sensors and updates vectors
 * @param  acs: a pointer to an existing Attitude Control System object
 * @param  hi2c: i2c handle for the IMU
 * @return Updates gyro_vector, mag_vector, solar_vector, and sun_status in acs
*/
void readSensors(ACSType* acs, SPI_HandleTypeDef* hspi);


#endif /* SENSORS_H */
