/**
  ******************************************************************************
  * @file           : OrientationController.c
  * @brief          : Implementation for the reorientation state of the ACS
  ******************************************************************************
  **
	* 
	* Created by Galen Savidge. Edited 4/21/2019.
  ******************************************************************************
  */

#include "FeedbackControl.h"


// Angular speed portion
#define K 1
#define KP (K*0.05)
#define KD (K*0.1)

#define ORIENTATION_W_MAG -0.017 // About 1 deg/s in rad/s


void runOrientationController(ACSType* acs, float dt, int first_step) {
	static int init_run = 0;
	Matrix w_err, last_w_err, P, D, wdot_desired, rw_pwm;
	
	if(init_run == 0) {
		w_err = newMatrix(3, 1);
		last_w_err = newMatrix(3, 1);
		P = newMatrix(3, 1);;
		D = newMatrix(3, 1);
		wdot_desired = newMatrix(3, 1);
		rw_pwm = newMatrix(3, 1);
		init_run = 1;
	}
	
	if(dt == 0) {
		return;
	}
	
	if(first_step) {
		vectorSetXYZ(wdot_desired, 0, 0, 0);
	}
	else {
		// Find error in rotation speed
		matrixCopy(acs->z_err, w_err);
		float z_err_norm = vectorNorm(w_err);
		matrixScale(w_err, ORIENTATION_W_MAG/z_err_norm);
		matrixSubtract(acs->gyro_vector, w_err, w_err); // w_err = w - (-0.017*z_err/norm(z_err))
		
		// Proportional component
		matrixCopy(w_err, P);
		matrixScale(P, KP);
		
		// Derivative component
		matrixCopy(w_err, D);
		matrixSubtract(D, last_w_err, D);
		matrixScale(D, KD/dt);
		
		// Sum P and D
		matrixAdd(P, D, wdot_desired);
		matrixScale(wdot_desired, -1);
	}
	
	// Record last error
	matrixCopy(w_err, last_w_err);
	
	// Find reaction wheel PWM
	wdot2rw_pwm(acs, wdot_desired, rw_pwm, dt);
	
	// Set actuator output here
}
