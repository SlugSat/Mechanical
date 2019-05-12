/**
  ******************************************************************************
  * @file           AttitudeEstimation.h
  * @brief          Sensor fusion and attitude determination
  ******************************************************************************
  ** This module contains the code to run closed loop gyro integration using
  * feedback from the magnetometer and solar vector. Call initalizeACS() in 
  * ACS.h to initialize an Attitude Control System struct, then pass it into
  * updateAttitudeEstimate() every loop to get the craft's Direction Consine
  * Matrix (DCM).
  * 
  * Created by Galen Savidge. Edited 5/12/2019.
  ******************************************************************************
  */

// LIBRARIES
#include <AttitudeEstimation.h>
#include <ReferenceFrames.h>
#include <math.h>


// CONSTANTS
#define KP_MAG_BASE 0.1 //1.0
#define KI_MAG_BASE 0.03 //0.3
#define KP_SV_BASE 0.2 //1.0
#define KI_SV_BASE 0.06 //0.3


// HELPER FUNCTIONS
/** 
 * @brief  Finds the exponential Rodrigues parameter form of an angular velocity vector
 * @param  w: a 3x1 angular velocity column vector
 * @param  Rexp: an allocated 3x3 Matrix to hold the exponential Matrix
 * @return None
*/
void findRexp(Matrix w, Matrix Rexp);

/**
 * @brief Sinc function that handles the case where x ~= 0
*/
float sinc(float x);

/** 
 * @brief  Performs closed loop integration on the given DCM using the Rexp form
 * @param  acs: an initialized ACS struct which has up-to-date sensor data
 * @param  K*: feedback constants for the mag and solar vectors
 * @param  dt: time in seconds since last call of this function
 * @return None
*/
void integrateDCM(ACSType* acs, float Kp_mag, float Ki_mag, float Kp_sv, float Ki_sv, float dt);


// PUBLIC FUNCTIONS

Matrix initializeDCM(float yaw, float pitch, float roll) {
	Matrix r = newMatrix(3, 3);
	
	// Angles in radians
	float y_r = PI*yaw/180;
	float p_r = PI*pitch/180;
	float r_r = PI*roll/180;
	
	makeDCM_zyx(r, y_r, p_r, r_r);
	
	return r;
}


void updateAttitudeEstimate(ACSType* acs) {
	if(acs->dt == 0) {
		return;
	}
	
	float Kp_mag, Ki_mag, Kp_sv, Ki_sv;
	
	Kp_mag = KP_MAG_BASE;
	Ki_mag = KI_MAG_BASE;
	
	if(acs->sun_status == SV_FOUND) {
		Kp_sv = KP_SV_BASE;
		Ki_sv = KI_SV_BASE;
	}
	else {
		Kp_sv = 0;
		Ki_sv = 0;
	}
	
	integrateDCM(acs, Kp_mag, Ki_mag, Kp_sv, Ki_sv, acs->dt);
}


void integrateDCM(ACSType* acs, float Kp_mag, float Ki_mag, float Kp_sv, float Ki_sv, float dt) {
			
	static char init_run = 0;
	static Matrix mag_normalized, sv_normalized, mi_normalized, svi_normalized;
	static Matrix mag_i_body, sv_i_body; // Inertial sensor vectors translated to body
	static Matrix mag_err, sv_err; // Error vectors (result of cross product between measured and inertial)
	static Matrix merr_x_Kp, sverr_x_Kp; // Error multiplied by Kp
	static Matrix gyro_with_bias; // (Gyro vector) - (bias) + (Kp terms)
	static Matrix bdot; // (-Ki terms)
	static Matrix Rexp;
	static Matrix new_R;
			
	if(init_run == 0) { // Initialize local Matrix objects on first run
		mag_normalized = newMatrix(3, 1);
		sv_normalized = newMatrix(3, 1);
		mi_normalized = newMatrix(3, 1);
		svi_normalized = newMatrix(3, 1);
		mag_i_body = newMatrix(3, 1);
		sv_i_body = newMatrix(3, 1);
		mag_err = newMatrix(3, 1);
		sv_err = newMatrix(3, 1);
		merr_x_Kp = newMatrix(3, 1);
		sverr_x_Kp = newMatrix(3, 1);
		gyro_with_bias = newMatrix(3, 1);
		bdot = newMatrix(3, 1);
		Rexp = newMatrix(3, 3);
		new_R = newMatrix(3, 3);
		
		init_run = 1;
	}
	
	// ***** NORMALIZE MAG, SOLAR VECTOR, AND INERTIAL VECTORS ******
	float norm_mag = vectorNorm(acs->mag_vector);
	float norm_sv = vectorNorm(acs->solar_vector);
	float norm_mi = vectorNorm(acs->mag_inertial);
	float norm_svi = vectorNorm(acs->sv_inertial);

	if(norm_mag == 0 || norm_sv == 0 || norm_mi == 0 || norm_svi == 0) {
		// Need better error handling
		return;
	}
	
	matrixCopy(acs->mag_vector, mag_normalized);
	matrixCopy(acs->solar_vector, sv_normalized);
	matrixCopy(acs->mag_inertial, mi_normalized);
	matrixCopy(acs->sv_inertial, svi_normalized);
	matrixScale(mag_normalized, 1.0/norm_mag);
	matrixScale(sv_normalized, 1.0/norm_sv);
	matrixScale(mi_normalized, 1.0/norm_mi);
	matrixScale(svi_normalized, 1.0/norm_svi);
	
	// ***** FIND ERROR FROM MAG *****
	matrixMult(acs->Rt, mi_normalized, mag_i_body); // Translate mag to body
	vectorCrossProduct(mag_normalized, mag_i_body, mag_err);
	matrixCopy(mag_err, merr_x_Kp);
	matrixScale(merr_x_Kp, Kp_mag); // Kp_mag * mag_err
	
	// ***** FIND ERROR FROM SOLAR VECTOR *****
	matrixMult(acs->Rt, svi_normalized, sv_i_body); // Translate mag to body
	vectorCrossProduct(sv_normalized, sv_i_body, sv_err); // Cross product
	matrixCopy(sv_err, sverr_x_Kp);
	matrixScale(sverr_x_Kp, Kp_sv); // Kp_sv * mag_err
	
	// ***** ADD FEEDBACK TO GYRO *****
	matrixSubtract(acs->gyro_vector, acs->gyro_bias, gyro_with_bias);
	matrixAdd(gyro_with_bias, merr_x_Kp, gyro_with_bias); // Add mag err*Kp
	matrixAdd(gyro_with_bias, sverr_x_Kp, gyro_with_bias); // Add sv err*Kp
	
	// ***** CALCULATE NEW BIAS ESTIMATE *****
	matrixScale(mag_err, -Ki_mag);
	matrixScale(sv_err, -Ki_sv);
	matrixCopy(mag_err, bdot); // bdot equals -Ki_mag*mag_err
	matrixAdd(bdot, sv_err, bdot); // bdot now equals -Ki_mag*mag_err - Ki_sv*sv_err
	acs->gyro_bias_dot_norm = vectorNorm(bdot);
	matrixScale(bdot, dt); // Multiply bdot*dt
	matrixAdd(acs->gyro_bias, bdot, acs->gyro_bias); // bias = bias + dt*bdot
	
	// ***** INTEGRATE DCM *****
	matrixScale(gyro_with_bias, dt);
	findRexp(gyro_with_bias, Rexp); // Rexp = findRexp(gyro_with_bias*dt)
	matrixMult(acs->R, Rexp, new_R); // new_R = R*Rexp(gyro_with_bias*dt)
	matrixCopy(new_R, acs->R); // R = R*Rexp(gyro_with_bias*dt)
	
	// ***** TRANSPOSE DCM *****
	matrixTranspose(acs->R, acs->Rt);
}


void findEulerAngles(Matrix R, float* yaw_pitch_roll) {
	static char init_run = 0;
	static Matrix Rt;
	
	if(init_run == 0) {
		Rt = newMatrix(3, 3);
		init_run = 1;
	}
	
	// Find transpose of R
	matrixTranspose(R, Rt);
	
	// Yaw
	yaw_pitch_roll[0] = atan2(matrixGetElement(Rt, 1, 2), matrixGetElement(Rt, 1, 1));

	// Pitch
	yaw_pitch_roll[1] = asin(-matrixGetElement(Rt, 1, 3));

	// Roll
	yaw_pitch_roll[2] = atan2(matrixGetElement(Rt, 2, 3), matrixGetElement(Rt, 3, 3));
}

// Helper function to find the sinc of a float
float sinc(float x) {
	// Taylor portion for small x
	if(fabsf(x) <= 0.5) {
		return pow(x,6)/120.0 - pow(x,2)/6.0 + 1.0;
	}  
	// sinx/x works for larger x
	else {
		return sin(x)/x;
	}
}


// HELPER FUNCTIONS

void findRexp(Matrix w, Matrix Rexp) {
	static int init_run = 0;
	static Matrix rx, rx_2;
	
	if(!init_run) {
		rx = newMatrix(3, 3);
		rx_2 = newMatrix(3, 3);
		init_run = 1;
	}
	
	float wnorm = vectorNorm(w);
	vectorRcross(w, rx);
	float s = sinc(wnorm/2);
	float c = cos(wnorm/2);
	matrixMult(rx, rx, rx_2); // rx*rx
	matrixScale(rx_2, s*s/2); // rx*rx*s*s/2
	matrixScale(rx, s*c);     // rx*s*c
	matrixAdd(rx, rx_2, Rexp); // Rexp = rx + rx_2

	// Add I(3) to Rexp
	matrixSet(Rexp, 1, 1, matrixGetElement(Rexp, 1, 1) + 1);
	matrixSet(Rexp, 2, 2, matrixGetElement(Rexp, 2, 2) + 1);
	matrixSet(Rexp, 3, 3, matrixGetElement(Rexp, 3, 3) + 1);
}
