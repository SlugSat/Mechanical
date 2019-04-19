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


// PRIVATE FUNCTION
void wdot2rw_pwm(ACSType* acs, float* rw_pwm, float dt);


// PUBLIC FUNCTIONS
void findErrorVectors(ACSType* acs) {
	static float init_run = 0;
	static Matrix zhat_B, xhat_B; // Body frame unit vectors
	
	if(init_run == 0) {
		zhat_B = make3x1Vector(0, 0, 1);
		xhat_B = make3x1Vector(1, 0, 0);
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
