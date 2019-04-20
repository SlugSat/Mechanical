/**
  ******************************************************************************
  * @file           : StabilizationController.c
  * @brief          : Implementation for the stabilzation state of the ACS
  ******************************************************************************
  ** 
	* As of now these functions change actuator outputs inside of the function
	* itself but that is subject to change.
	* 
	* Created by Galen Savidge. Edited 4/19/2019.
  ******************************************************************************
  */

#include "FeedbackControl.h"


// Angular speed portion
#define K_WDOT 0.3
#define KP_WDOT (K_WDOT*0.006)
#define KD_WDOT (K_WDOT*0.4)

// Torque portion
#define K_T 0.0005;
#define KP_T (K_T*1.5)
#define KI_T (K_T*0.05)
#define KD_T (K_T*8)


void runStabilizationController(ACSType* acs, float dt, int first_step) {
	static int init_run = 0;
	Matrix torque_integrator, last_err;
	
	if(init_run == 0) {
		torque_integrator = make3x1Vector(0, 0, 0);
		last_err = newMatrix(3, 1);
		init_run = 1;
	}
	
	if(first_step) {
		vectorSetXYZ(torque_integrator, 0, 0, 0);
		// Set wdot_desired to [0, 0, 0]
	}
	else {
		// Controller goes here
	}
	
	matrixCopy(acs->err, last_err);
	
	// Set actuator output
}
