/**
  ******************************************************************************
  * @file           : FeedbackControl.c
  * @brief          : Contains implementations for the feedback control states in the ACS
  ******************************************************************************
  ** 
	* As of now these functions change actuator outputs inside of the function
	* itself but that is subject to change.
	* 
	* Created by Galen Savidge. Edited 4/18/2019.
  ******************************************************************************
  */

// HEADER FILE
#include <FeedbackControl.h>
#include <math.h>


// PRIVATE FUNCTION
void wdot2rw_pwm(ACSType* acs, float* rw_pwm, float dt);


// PUBLIC FUNCTIONS
void findErrorVectors(ACSType* acs) {
	static float init_run = 0;
	static Matrix zhat_B, corner_B; // Body frame vectors
	
	if(init_run == 0) {
		zhat_B = make3x1Vector(0, 0, 1);
		corner_B = newMatrix(3, 1);
		init_run = 1;
	}
	
	
}


void runBdotController(ACSType* acs, float dt) {
	// this is a comment and another one
	return;
}


void runOrientationController(ACSType* acs, float dt) {
	return;
}


void runStabilizationController(ACSType* acs, float dt) {
	return;
}

void findSunInertial(ACSType* acs, double julianDate){
	double num_days = julianDate - 2451545.0; // number of days since 1 Jan 2000.
	float mean_longitude = fmod(280.46 + 0.9856474*num_days, 360.0);
	float mean_anomoly = fmod(357.528 + 0.9856003*num_days, 360.0);
	
	//Ecliptic Longitude:
	float lambda = mean_longitude + 1.915*sin(mean_anomoly*PI/180) + 0.020*sin(2*mean_anomoly*PI/180);
	
	matrixSet(acs->sv_inertial, 1, 1, cos(lambda));
	matrixSet(acs->sv_inertial, 2, 1, sin(lambda));
	matrixSet(acs->sv_inertial, 3, 1, 0);
	
	return;
}
