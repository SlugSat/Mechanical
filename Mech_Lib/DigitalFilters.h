/**
  ******************************************************************************
  * @file           : DigitalFilters.h
  * @brief          : Header for the DigitalFilters module.
  ******************************************************************************
  ** Use this module to digitally filter sensor data.
	*
	* Created by Galen Savidge. Edited 3/4/2019.
  ******************************************************************************
  */

#ifndef DIGITALFILTERS_H
#define	DIGITALFILTERS_H

/* Public types ------------------------------------------------------------*/
typedef struct _MovingAvgFilter* MovingAvgFilter;

/* Public functions prototypes ---------------------------------------------*/

/** 
 * @brief  Allocates a new MovingAvgFilter; initializes history to 0
 * @param  Number of sensors and number of readings per sensor to average
 * @return Pointer to the new MovingAvgFilter
*/
MovingAvgFilter newMovingAvgFilter(char num_sensors, char num_readings);

/** 
 * @brief  Runs the MovingAvgFilter with new data and returns filtered result
 * @param  f: An allocated MovingAvgFilter
 * @param  new_readings: A num_sensors size array of raw data. Contains filtered data after the function returns.
 * @return None
*/
void runMovingAvgFilter(MovingAvgFilter f, float* new_readings);

#endif
