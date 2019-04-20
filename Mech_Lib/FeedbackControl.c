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


// CONSTANTS
#define DIV_ROOT2 ((float)0.70710678118)


// PRIVATE FUNCTIONS
void wdot2rw_pwm(ACSType* acs, float* rw_pwm, float dt);

float sign(float x);

// PUBLIC FUNCTIONS
void findErrorVectors(ACSType* acs) {
	static float init_run = 0;
	static Matrix zhat_B, craft_B, n_I, n_B, corner_B;
	
	if(init_run == 0) {
		zhat_B = make3x1Vector(0, 0, 1);
		corner_B = newMatrix(3, 1);
		craft_B = newMatrix(3, 1);
		init_run = 1;
	}
	
	// ***** FIND POINTING ERROR BETWEEN Z AXIS AND CRAFT POSITION VECTOR *****
	matrixMult(acs->Rt, acs->craft_inertial, craft_B); // Earth->craft vector in body frame
	vectorCrossProduct(craft_B, zhat_B, acs->z_err);
	matrixScale(acs->z_err, 0.5);
	
	// ***** FIND ROTATION ERROR BETWEEN N VECTOR AND CRAFT CORNER VECTOR *****
	// Find the unit vector normal to the plane containing the Earth, craft, and Sun
	vectorCrossProduct(acs->craft_inertial, acs->sv_inertial, n_I);
	float n_I_norm = vectorNorm(n_I);
	matrixScale(n_I, 1.0/n_I_norm);
	matrixMult(acs->Rt, n_I, n_B); // Translate to body frame
	
	// Make corner_B vector point to the edge closest to n_B (roughly)
	matrixSet(corner_B, 1, 1, DIV_ROOT2*sign(matrixGetElement(n_B, 1, 1)));
	matrixSet(corner_B, 2, 1, DIV_ROOT2*sign(matrixGetElement(n_B, 2, 1)));
	// corner_B.z = 0 already
	
	// Take cross product to find error
	vectorCrossProduct(n_B, corner_B, acs->n_err);
	matrixScale(acs->n_err, 0.5);
}


void runBdotController(ACSType* acs, float dt) {
	// this is a comment
	return;
}


void runOrientationController(ACSType* acs, float dt) {
	return;
}


void runStabilizationController(ACSType* acs, float dt) {
	return;
}

float sign(float x) {
	if(x < 0) {
		return -1.0;
	}
	else {
		return 1.0;
	}
}