/**
  ******************************************************************************
  * @file           : InertialVectors.h
  * @brief          : Contains functions to find sun and magnetic field inertial vectors
  ******************************************************************************
	* Created by Galen Savidge. Edited 5/4/2019.
  ******************************************************************************
  */

// HEADER FILE
#include <InertialVectors.h>
#include <math.h>
#include <ReferenceFrames.h>
#include <IGRF/mag_inertial.h>


void findSunInertial(ACSType* acs){
	double num_days = acs->julian_date - 2451545.0; // number of days since 1 Jan 2000.
	float mean_longitude = fmod(280.46 + 0.9856474*num_days, 360.0);
	float mean_anomoly = fmod(357.528 + 0.9856003*num_days, 360.0);
	
	//Ecliptic Longitude:
	float lambda = mean_longitude + 1.915*sin(mean_anomoly*PI/180) + 0.020*sin(2*mean_anomoly*PI/180);
	
	matrixSet(acs->sv_inertial, 1, 1, cos(lambda));
	matrixSet(acs->sv_inertial, 2, 1, sin(lambda));
	matrixSet(acs->sv_inertial, 3, 1, 0);
	
	return;
}


void findMagInertial(ACSType* acs) {
}
