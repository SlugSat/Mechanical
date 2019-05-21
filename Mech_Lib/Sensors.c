/*
  ******************************************************************************
  * @file           Sensors.c
  * @brief          Contains functions to read from the physical sensors on the satellite
  ******************************************************************************
  * Created by Galen Savidge. Edited 5/12/2019.
  ******************************************************************************
  */


// LIBRARIES
#include <Sensors.h>
#include <UM7_reg.h>
#include <SolarVectors.h>
#include <math.h>


// CONSTANTS
#define MAG_HIST_LENGTH 10
#define SV_HIST_LENGTH 10

#define SENSOR_READ_DELAY_MS 50


// PUBLIC FUNCTIONS

void initializeSensors(ACSType* acs, SPI_HandleTypeDef* hspi, ADC_HandleTypeDef* hadc) {
	// Sensor hardware
	UM7_Init(hspi, 100, 100);
	HAL_ADC_Start_DMA(hadc, acs->sv_raw, NUM_SOLAR_PANELS);
	
	// Sensor moving average filters
	acs->mag_filter = newMovingAvgFilter(3, MAG_HIST_LENGTH);
	acs->sv_filter = newMovingAvgFilter(NUM_SOLAR_PANELS, SV_HIST_LENGTH);
	
	// Read sensors until moving average filters are full
	uint8_t sensor_iterations = SV_HIST_LENGTH > MAG_HIST_LENGTH ? SV_HIST_LENGTH : MAG_HIST_LENGTH;
	for(int i = 0;i < sensor_iterations;i++) {
		readSensors(acs, hspi);
		HAL_Delay(SENSOR_READ_DELAY_MS);
	}
}


void readSensors(ACSType* acs, SPI_HandleTypeDef* hspi) {
	static float gyro_data[3], mag_data[3], sv_data[NUM_SOLAR_PANELS];
	
	// Read gyro and transform into a column vector Matrix
	get_gyr_data(hspi, gyro_data);
	vectorCopyArray(acs->gyro_vector, gyro_data, 3);
	
	// Read magnetometer, iterate moving average filter, transform into a vector Matrix
	get_mag_data(hspi, mag_data);
	runMovingAvgFilter(acs->mag_filter, mag_data);
	vectorCopyArray(acs->mag_vector, mag_data, 3);
	
	// Read solar vector
	for(int i = 0;i < NUM_SOLAR_PANELS;i++) {
		sv_data[i] = ADC_TO_VOLTS(acs->sv_raw[i]);
	}
	runMovingAvgFilter(acs->sv_filter, sv_data);
	acs->sun_status = findSolarVector(sv_data, NUM_SOLAR_PANELS, acs->solar_vector);
}
