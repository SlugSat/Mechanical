/**
  ******************************************************************************
  * @file           FeedbackControl.h
  * @brief          Contains implementations for the feedback control states in the ACS
  ******************************************************************************
  ** Controller implementations are in OrientationController.c and
  * StabilizationController.c. A call of findErrorVectors() is required before 
  * either runOrientationController() or runStabilizationController() for the 
  * ACS to work correctly.
  * 
  * Created by Galen Savidge. Edited 4/21/2019.
  ******************************************************************************
  */

// HEADER FILE
#include <FeedbackControl.h>
#include <math.h>


// CONSTANTS
#define DIV_ROOT2 ((float)0.70710678118)

#define V_RAIL 8.0 // Volts

// Reaction wheel motor constants
#define KT 0.00713 // Nm/A
#define KE 0.00713332454 // V/rad/s
#define R 92.7 // Ohms
#define C0 19e-6 // Static friction torque (Nm)
#define CV 20.94e-9 // Dynamic friction torque (Nm/rad/s)

// Torque rod moment
#define MAX_MOMENT 2.0



// PRIVATE FUNCTIONS
float sign(float x);


// PUBLIC FUNCTIONS
void findErrorVectors(ACSType* acs) {
	static int init_run = 0;
	static Matrix zhat_B, craft_B, n_I, n_B, corner_B, sv_B;
	
	if(init_run == 0) {
		zhat_B = make3x1Vector(0, 0, 1);
		craft_B = newMatrix(3, 1);
		n_I = newMatrix(3, 1);
		n_B = newMatrix(3, 1);
		corner_B = newMatrix(3, 1);
		sv_B = newMatrix(3, 1);
		init_run = 1;
	}
	
	// ***** FIND POINTING ERROR BETWEEN Z AXIS AND CRAFT POSITION VECTOR *****
	matrixMult(acs->Rt, acs->craft_inertial, craft_B); // Find Earth->craft vector in body frame
	vectorCrossProduct(craft_B, zhat_B, acs->z_err);
	matrixScale(acs->z_err, 0.5);
	
	// ***** FIND ROTATION ERROR BETWEEN N VECTOR AND CRAFT CORNER VECTOR *****
	// Find the unit vector normal to the plane containing the Earth, craft, and Sun
	vectorCrossProduct(acs->craft_inertial, acs->sv_inertial, n_I);
	float n_I_norm = vectorNorm(n_I);
	if(n_I_norm != 0) {
		matrixScale(n_I, 1.0/n_I_norm);
	}
	matrixMult(acs->Rt, n_I, n_B); // Translate to body frame
	
	// Make corner_B vector point to the edge closest to n_B (roughly)
	vectorSetXYZ(corner_B, DIV_ROOT2*sign(matrixGetElement(n_B, 1, 1)), DIV_ROOT2*sign(matrixGetElement(n_B, 2, 1)), 0);
	
	// Take cross product to find error
	vectorCrossProduct(n_B, corner_B, acs->n_err);
	matrixScale(acs->n_err, 0.5);
	
	
	// Sum Z and N error to find total error
	matrixAdd(acs->z_err, acs->n_err, acs->err);
	
	
	// Find scalar pointing error (used for state transitions)
	acs->pointing_err = acos(vectorDotProduct(zhat_B, craft_B))*180/PI;
	
	// Find angle between zhat_B and sun
	matrixMult(acs->Rt, acs->sv_inertial, sv_B);
	acs->zb_sun_angle = acos(vectorDotProduct(zhat_B, sv_B))*180/PI;
}


void runBdotController(ACSType* acs) {
	static int init_run = 0;
	static Matrix b_rot, w_adj, bdot, last_mag;
	
	if(init_run == 0) {
		b_rot = newMatrix(3,1);
		w_adj = newMatrix(3,1);
		bdot = newMatrix(3,1);
		last_mag = newMatrix(3,1);
		init_run = 1;
	}
	
	// ***** FIND CHANGE OF MAGNETIC FIELD (ROTATION THROUGH MAG FIELD) *****
	vectorCrossProduct(last_mag, acs->mag_vector, b_rot); //b_rot = last_mag X mag_vector
	
	// ***** ADJUSTS OMEGA TO GET TRUE ROTATION AND FIND BDOT*****
	matrixSubtract(b_rot, acs->gyro_vector, w_adj);//w_adj = b_rot - w 
	vectorCrossProduct(w_adj, acs->mag_vector, bdot);
	
	// ***** FIND PWM FOR EACH TORQUE ROD *****
	float bdot_norm = vectorNorm(bdot);
	if(bdot_norm == 0) return;
	matrixScale(bdot, -100/(TR_MAXDIP*bdot_norm) );
	matrixCopy (bdot, acs->tr_PWM);
	
	return;
}


void wdot2rw_pwm(ACSType* acs, Matrix wdot_desired) {
	static int init_run = 0;
	static Matrix torque, p, p_rw, wxp, Jxwdot, w_rw_new, fr_torque;
	
	if(init_run == 0) {
		torque = newMatrix(3, 1);
		p = newMatrix(3, 1);
		p_rw = newMatrix(3, 1);
		wxp = newMatrix(3, 1);
		Jxwdot = newMatrix(3, 1);
		w_rw_new = newMatrix(3, 1);
		fr_torque = newMatrix(3, 1);
		init_run = 1;
	}
	
	// Find momentum vectors
	matrixMult(acs->J_body, acs->gyro_vector, p);
	matrixMult(acs->J_rw, acs->w_rw, p_rw);
	matrixAdd(p, p_rw, p); // p = J_B*w + J_rw*w_rw
	
	// Find total torque
	vectorCrossProduct(acs->gyro_vector, p, wxp); // wxp = w x (J_B*w + J_rw*w_rw)
	matrixMult(acs->J_body, wdot_desired, Jxwdot);
	matrixAdd(Jxwdot, wxp, torque); // wxp = (J_B*wdot) + w x (w*J_B + J_rw*w_rw)
	matrixScale(torque, -1);

	// Find desired rw_wdot
	matrixMult(acs->J_rw_inv, torque, w_rw_new); // w_rw_new = J_rw_inv*torque
	matrixScale(w_rw_new, acs->dt); // w_rw_new = J_rw_inv*torque*dt
	matrixAdd(w_rw_new, acs->w_rw, w_rw_new); // w_rw_new = w_rw + J_rw_inv*torque*dt
	
	// Add torque to account for motor friction
	for(int i = 1;i <= 3;i++) {
		float fr_mag = C0 + CV*fabsf(matrixGetElement(acs->w_rw, i, 1));
		matrixSet(fr_torque, i, 1, sign(matrixGetElement(w_rw_new, i, 1))*fr_mag);
	}
	matrixAdd(torque, fr_torque, torque);

	// Find PWM
	matrixScale(w_rw_new, KE);
	matrixScale(torque, R/KT);
	matrixAdd(w_rw_new, torque, acs->rw_PWM);
	matrixScale(acs->rw_PWM, 100.0/(V_RAIL));
}


float sign(float x) {
	if(x < 0) {
		return -1.0;
	}
	else {
		return 1.0;
	}
}
