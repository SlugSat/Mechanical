/**
  ******************************************************************************
  * @file           DigitalFilters.h
  * @brief          Filter sensor data
  ******************************************************************************
  ** Currently includes a moving average filter datatype, which runs a variable 
  * length digital moving average filter on a configurable number of sensor 
  * readings. Use newMovingAvgFilter() to allocate a MovingAvgFilter and then 
  * pass it to runMovingAvgFilter() each loop to filter sensor data.
  *
  * Created by Galen Savidge. Edited 5/12/2019.
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

#endif /* DIGITALFILTERS_H */
