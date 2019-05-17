/**
  ******************************************************************************
  * @file           StabilizationController.c
  * @brief          Implementation for the stabilzation state of the ACS
  ******************************************************************************
  * This controller includes momentum dumping and an integrator to remove steady
	* state bias.
	* 
	* Created by Galen Savidge. Edited 4/21/2019.
  ******************************************************************************
  */

#include "FeedbackControl.h"
#include <math.h>

// Angular speed controller portion
#define K_WDOT -0.3
#define KP_WDOT (K_WDOT*0.006)
#define KD_WDOT (K_WDOT*0.4)

// Torque controller portion
#define K_T 5e-4
#define KP_T (K_T*2)
#define KI_T (K_T*0.05)
#define KD_T (K_T*8)

// Torque rod
#define MAXDIP 2.0


/**
 * Convert torque into an angular acceleration based on the craft's physical
 * system. CURRENTLY ASSUMES A IS I3.
 */
void torque2wdot(ACSType* acs, Matrix torque_vector, Matrix wdot_vector);


void runStabilizationController(ACSType* acs, Matrix err, int first_step) {

	static int init_run = 0;
	static Matrix torque_integrator, last_err, P, I, D, controller_torque, controller_wdot, 
		wdot_desired, w_rw_biased, h_rw, m, trTorque;
	
	if(init_run == 0) {
		controller_torque = newMatrix(3, 1);
		torque_integrator = make3x1Vector(0, 0, 0);
		last_err = newMatrix(3, 1);
		P = newMatrix(3, 1);
		I = newMatrix(3, 1);
		D = newMatrix(3, 1);
		controller_wdot = newMatrix(3, 1);
		wdot_desired = newMatrix(3, 1);
		w_rw_biased = newMatrix(3, 1);
		h_rw = newMatrix(3, 1);
		m = newMatrix(3, 1);
		trTorque = newMatrix(3,1);
		init_run = 1;
	}
	
	if(acs->dt == 0) {
		return;
	}
	
	if(first_step) {
		vectorSetXYZ(torque_integrator, 0, 0, 0);
		vectorSetXYZ(wdot_desired, 0, 0, 0);
		vectorSetXYZ(acs->tr_PWM, 0, 0, 0);
	}
	else {


		// ***** MOMENTUM DUMPING *****
		//Variables
		int i;

		//Determine available torque from torque rods
		matrixCopy(acs->w_rw, w_rw_biased);
		matrixAddScalar(w_rw_biased, -RW_BASE_SPEED); // Subtract base speed from each element in w_rw
		matrixMult(acs->J_rw, w_rw_biased, h_rw); //Reaction wheel momentum
		float normMom = vectorNorm(h_rw);  //Find the Norm
		
		if (normMom != 0){
			matrixScale(h_rw, 1.0/normMom); // Normalize h_rw
			vectorCrossProduct(h_rw, acs->mag_vector, m); // Dipole moment
			matrixScale(m, 1000); // Scale dipole by 1000
			
			//Ensure dipole moment stays within bounds
			for(i=0;i<3;i++){
				if(matrixGetElement(m, i, 1) > MAXDIP){
					matrixSet(m, i, 1, MAXDIP);
				}
				else if (matrixGetElement(m, i, 1) < -MAXDIP){
					matrixSet(m, i, 1, -MAXDIP);
				}
			}
			
			//Torque rod torque
			vectorCrossProduct(m, acs->mag_vector, trTorque);
		
			//Check reaction wheels angular velocity
			/*for(i = 1; i <= 3; i++){				
				//Turn on torque rods when reaction wheels are greater than 1000 RPM
				if(fabs(matrixGetElement(acs->w_rw, i, 1)) < 100) // || matrixGetElement(trTorque, i, 1) < 1e-6)
				{
					matrixSet(trTorque, i, 1, 0);
				}
			}*/
			
			// ***** FIND PWM FOR EACH TORQUE ROD *****
			//vectorCrossProduct(acs->mag_vector, trTorque, m);
			matrixCopy(m, acs->tr_PWM); 
			matrixScale(acs->tr_PWM, 100.0/MAXDIP);	//Scale to PWM
		}
	
		else
		{
			vectorSetXYZ(acs->tr_PWM, 0, 0, 0);
			vectorSetXYZ(trTorque, 0, 0, 0);
		}
		

		// ***** TORQUE CONTROLLER *****
		// Proportional component
		matrixCopy(err, P);
		matrixScale(P, KP_T);
		
		// Integrator component
		matrixCopy(err, I);
		matrixScale(I, KI_T*acs->dt);
		matrixAdd(torque_integrator, I, torque_integrator); // Adding in place works; multiplying in place doesn't
		
		// Derivative component
		matrixCopy(err, D);
		matrixSubtract(D, last_err, D);
		matrixScale(D, KD_T/acs->dt);
		
		// Sum the P, I, and D components
		matrixAdd(P, torque_integrator, controller_torque);
		matrixAdd(controller_torque, D, controller_torque);
		
		//Add Torque rod torque
		matrixAdd(controller_torque, trTorque, controller_torque);
		
		
		// ***** WDOT CONTROLLER *****
		// Proportional component
		matrixCopy(err, P);
		matrixScale(P, KP_WDOT);
		
		// Derivative component
		matrixCopy(err, D);
		matrixSubtract(D, last_err, D);
		matrixScale(D, KD_WDOT/acs->dt);
		
		// Sum the P and D components
		matrixAdd(P, D, controller_wdot);
		
		// Sum torque and wdot controller components
		torque2wdot(acs, controller_torque, wdot_desired);
		matrixAdd(wdot_desired, controller_wdot, wdot_desired);
	}
	
	// Record last error
	matrixCopy(err, last_err);
	
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
