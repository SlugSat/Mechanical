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
	
	vectorSetXYZ(acs->sv_inertial, cos(lambda), sin(lambda), 0);
	
	return;
}


void findMagInertial(ACSType* acs) {
	static int init_run = 0;
	static Matrix igrf_ned, igrf_j2000;
	
	if(init_run == 0) {
		igrf_ned = newMatrix(3, 1);
		igrf_j2000 = newMatrix(3, 1);
		init_run = 1;
	}
	
	// Find longitude, latitude, and altitude
	J2000_2_LongLatAlt(acs->craft_j2000, acs->julian_date, &acs->longitude, &acs->latitude, &acs->altitude);
	
	// Run IGRF
	get_mag_inertial(acs->julian_date, acs->longitude, acs->latitude, acs->altitude, igrf_ned);
	
	// Convert result to ecliptic frame
	NED_2_J2000(igrf_ned, acs->craft_j2000, acs->julian_date, igrf_j2000);
	J2000_2_ecliptic(igrf_j2000, acs->mag_inertial);
}
