/**
  ******************************************************************************
  * @file           : DigitalFilters.h
  * @brief          : Source file for the DigitalFilters module.
  ******************************************************************************
  ** Use this module to digitally filter sensor data.
	*
	* Created by Galen Savidge. Edited 3/4/2019.
  ******************************************************************************
  */

#include <DigitalFilters.h>
#include <stdlib.h>

struct _MovingAvgFilter {
	float** readings; // Size: num_readings by num_sensors
	char num_readings;
	char num_sensors;
	char index; // In range [0, num_readings)
	float* last_avgs; // Size: num_sensors
};

MovingAvgFilter newMovingAvgFilter(char num_sensors, char num_readings) {
	MovingAvgFilter f = malloc(sizeof(struct _MovingAvgFilter));
	
	// Initialize internal variables
	f->num_sensors = num_sensors;
	f->num_readings = num_readings;
	f->index = 0;
	f->last_avgs = calloc(num_sensors, sizeof(float));
	
	f->readings = calloc(num_sensors, sizeof(float*));
	for(int i = 0;i < num_sensors;i++) {
		f->readings[i] = calloc(num_readings, sizeof(float));
	}
	
	return f;
}

void runMovingAvgFilter(MovingAvgFilter f, float* new_readings) {
	// Iterate through sensors
	for(int i = 0;i < f->num_sensors;i++) {
		f->last_avgs[i] -= f->readings[i][f->index]; // Subtract first reading
		f->readings[i][f->index] = new_readings[i]/(f->num_readings); // Record new reading
		f->last_avgs[i] += f->readings[i][f->index]; // Add new reading
		new_readings[i] = f->last_avgs[i]; // Return new average
	}
	
	// Increment index
	f->index++;
	f->index = f->index % f->num_readings;
}
