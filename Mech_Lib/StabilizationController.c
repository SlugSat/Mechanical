/**
  ******************************************************************************
  * @file           : StabilizationController.c
  * @brief          : Implementation for the stabilzation state of the ACS
  ******************************************************************************
  * 
	* 
	* Created by Galen Savidge. Edited 4/21/2019.
  ******************************************************************************
  */

#include "FeedbackControl.h"


// Angular speed portion
#define K_WDOT -0.3
#define KP_WDOT (K_WDOT*0.006)
#define KD_WDOT (K_WDOT*0.4)

// Torque portion
#define K_T 5e-4
#define KP_T (K_T*1.5)
#define KI_T (K_T*0.05)
#define KD_T (K_T*8)

/**
 * Convert torque into an angular acceleration based on the craft's physical
 * system. CURRENTLY ASSUMES A IS I3.
 */
void torque2wdot(ACSType* acs, Matrix torque_vector, Matrix wdot_vector);


void runStabilizationController(ACSType* acs, int first_step) {
	static int init_run = 0;
	static Matrix torque_integrator, last_err, P, I, D, controller_torque, controller_wdot, wdot_desired;
	
	if(init_run == 0) {
		torque_integrator = make3x1Vector(0, 0, 0);
		last_err = newMatrix(3, 1);
		P = newMatrix(3, 1);
		I = newMatrix(3, 1);
		D = newMatrix(3, 1);
		controller_torque = newMatrix(3, 1);
		controller_wdot = newMatrix(3, 1);
		wdot_desired = newMatrix(3, 1);
		init_run = 1;
	}
	
	if(acs->dt == 0) {
		return;
	}
	
	if(first_step) {
		vectorSetXYZ(torque_integrator, 0, 0, 0);
		vectorSetXYZ(wdot_desired, 0, 0, 0);
	}
	else {
		// ***** TORQUE CONTROLLER *****
		// Proportional component
		matrixCopy(acs->err, P);
		matrixScale(P, KP_T);
		
		// Integrator component
		matrixCopy(acs->err, I);
		matrixScale(I, KI_T*acs->dt);
		matrixAdd(torque_integrator, I, torque_integrator); // Adding in place works; multiplying in place doesn't
		
		// Derivative component
		matrixCopy(acs->err, D);
		matrixSubtract(D, last_err, D);
		matrixScale(D, KD_T/acs->dt);
		
		// Sum the P, I, and D components
		matrixAdd(P, I, controller_torque);
		matrixAdd(controller_torque, D, controller_torque);
		
		// ***** WDOT CONTROLLER *****
		// Proportional component
		matrixCopy(acs->err, P);
		matrixScale(P, KP_WDOT);
		
		// Derivative component
		matrixCopy(acs->err, D);
		matrixSubtract(D, last_err, D);
		matrixScale(D, KD_WDOT/acs->dt);
		
		// Sum the P and D components
		matrixAdd(P, D, controller_wdot);
		
		// Sum torque and wdot controller components
		torque2wdot(acs, controller_torque, wdot_desired);
		matrixAdd(wdot_desired, controller_wdot, wdot_desired);
	}
	
	// Record last error
	matrixCopy(acs->err, last_err);
	
	// Find reaction wheel PWM
	wdot2rw_pwm(acs, wdot_desired);
}


void torque2wdot(ACSType* acs, Matrix torque_vector, Matrix wdot_vector) {
	static int init_run = 0;
	static Matrix p, p_rw; // Momentum vectors
	static Matrix wxp; // w x (J_B*w + J_rw*w_rw)
	
	if(init_run == 0) {
		p = newMatrix(3, 1);
		p_rw = newMatrix(3, 1);
		wxp = newMatrix(3, 1);
		init_run = 1;
	}
	
	// Find momentum vectors
	matrixMult(acs->J_body, acs->gyro_vector, p);
	matrixMult(acs->J_rw, acs->w_rw, p_rw);
	matrixAdd(p, p_rw, p); // p = J_B*w + J_rw*w_rw
	
	// Find total torque
	vectorCrossProduct(acs->gyro_vector, p, wxp); // wxp = w x (J_B*w + J_rw*w_rw)
	
	matrixAdd(wxp, torque_vector, wxp);// wxp = torque + w x (J_B*w + J_rw*w_rw)
	matrixScale(wxp, -1.0); // wxp = -torque - w x (J_B*w + J_rw*w_rw)
	
	// Find wdot
	matrixMult(acs->J_body_inv, wxp, wdot_vector); // wdot = J_B_inv*(-torque - w x (J_B*w + J_rw*w_rw))
}
