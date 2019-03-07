/**
  ******************************************************************************
  * @file           : AttitudeEstimation.h
  * @brief          : Source file for the AttitudEstimation module.
  ******************************************************************************
  ** This module contains the code to run closed loop integration of the gyro
	* using feedback from the magnetometer and solar vector.
	* 
	* Created by Galen Savidge. Edited 3/6/2019.
  ******************************************************************************
  */

#include <AttitudeEstimation.h>
#include <math.h>
#include <main.h>

#define PI 3.141592654

// Helper functions
/** 
 * @brief  Finds the exponential Rodrigues parameter form of an angular velocity vector
 * @param  w: a 3x1 angular velocity column vector
 * @param  Rexp: an allocated 3x3 Matrix to hold the exponential Matrix
 * @return None
*/
void findRexp(Matrix w, Matrix Rexp);

// Sinc function that handles the case where x ~= 0
float	sinc(float x);

/** 
 * @brief  Allocates and initializes a 3x3 DCM Matrix
 * @param  Euler angles for the initial DCM (in degrees)
 * @return The new DCM Matrix
*/
Matrix initializeDCM(float yaw, float pitch, float roll) {
	Matrix r = newMatrix(3, 3);
	
	// Angles in radians
	float y_r = PI*yaw/180;
	float p_r = PI*pitch/180;
	float r_r = PI*roll/180;
	
	// From: https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Euler_angles_(z-y%E2%80%B2-x%E2%80%B3_intrinsic)_%E2%86%92_rotation_matrix
	
	// Row 1
	matrixSet(r, 1, 1, cos(p_r)*cos(y_r));
	matrixSet(r, 1, 2, -cos(r_r)*sin(y_r)+sin(r_r)*sin(p_r)*cos(y_r));
	matrixSet(r, 1, 3, sin(r_r)*sin(y_r)+cos(r_r)*sin(p_r)*cos(y_r));
	
	// Row 2
	matrixSet(r, 2, 1, cos(p_r)*sin(y_r));
	matrixSet(r, 2, 2, cos(r_r)*cos(y_r)+sin(r_r)*sin(p_r)*sin(y_r));
	matrixSet(r, 2, 3, -sin(r_r)*cos(y_r)+cos(r_r)*sin(p_r)*sin(y_r));
	
	// Row 3
	matrixSet(r, 3, 1, -sin(p_r));
	matrixSet(r, 3, 2, sin(r_r)*cos(p_r));
	matrixSet(r, 3, 3, cos(r_r)*cos(p_r));
	
	return r;
}

/** 
 * @brief  Performs closed loop integration on the given DCM using the Rexp form
 * @param  R: the DCM (initially returned from an initializeDCM() call)
 * @param  gyro: 
 * @param  mag: 
 * @param  sv: the solar vector from findSolarVector() in the SolarVectors module
 * @return None
*/
void integrateDCM(Matrix R, Matrix bias, Matrix gyro, Matrix mag, Matrix sv, 
		Matrix mag_inertial, Matrix sv_inertial, float Kp_mag, float Ki_mag,
		float Kp_sv, float Ki_sv, float dt) {
			
	static char init_run = 0;
	static Matrix Rt; // Transpose of the DCM
	static Matrix mag_i_body, sv_i_body; // Inertial sensor vectors translated to body
	static Matrix mag_rx, sv_rx; // rcross matrices of sensor vectors
	static Matrix mag_err, sv_err; // Error vectors (result of cross product between measured and inertial)
	static Matrix merr_x_Kp, sverr_x_Kp; // Error multiplied by Kp
	static Matrix gyro_with_bias; // (Gyro vector) - (bias) + (Kp terms)
	static Matrix bdot; // (-Ki terms)
	static Matrix Rexp;
	static Matrix new_R;
			
	if(init_run == 0) { // Initialize local Matrix objects on first run
		Rt = newMatrix(3, 3);
		mag_i_body = newMatrix(3, 1);
		sv_i_body = newMatrix(3, 1);
		mag_rx = newMatrix(3, 3);
		sv_rx = newMatrix(3, 3);
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
	float norm_mag = vectorNorm(mag);
	float norm_sv = vectorNorm(sv);
	float norm_mi = vectorNorm(mag_inertial);
	float norm_svi = vectorNorm(sv_inertial);

	if(norm_mag == 0 || norm_sv == 0 || norm_mi == 0 || norm_svi == 0) {
		// Need better error handling
		// printf("EULER ERROR: DIVIDE BY 0");
		while(1);
	}
	
	matrixScale(mag, 1.0/norm_mag);
	matrixScale(sv, 1.0/norm_sv);
	matrixScale(mag_inertial, 1.0/norm_mi);
	matrixScale(sv_inertial, 1.0/norm_svi);
	
	// ***** TRANSPOSE DCM *****
	matrixTranspose(R, Rt);
	
	// ***** FIND ERROR FROM MAG *****
	vectorRcross(mag, mag_rx);
	matrixMult(Rt, mag_inertial, mag_i_body); // Translate mag to body
	matrixMult(mag_rx, mag_i_body, mag_err); // Cross product
	matrixCopy(mag_err, merr_x_Kp);
	matrixScale(merr_x_Kp, Kp_mag); // Kp_mag * mag_err
	
	// ***** FIND ERROR FROM SOLAR VECTOR *****
	vectorRcross(sv, sv_rx);
	matrixMult(Rt, sv_inertial, sv_i_body); // Translate mag to body
	matrixMult(sv_rx, sv_i_body, sv_err); // Cross product
	matrixCopy(sv_err, sverr_x_Kp);
	matrixScale(sverr_x_Kp, Kp_sv); // Kp_sv * mag_err
	
	// ***** ADD FEEDBACK TO GYRO *****
	matrixSubtract(gyro, bias, gyro_with_bias);
	matrixAdd(gyro_with_bias, merr_x_Kp, gyro_with_bias); // Add mag err*Kp
	matrixAdd(gyro_with_bias, sverr_x_Kp, gyro_with_bias); // Add sv err*Kp
	
	// ***** CALCULATE NEW BIAS ESTIMATE *****
	matrixScale(mag_err, -Ki_mag);
	matrixScale(sv_err, -Ki_sv);
	matrixCopy(mag_err, bdot); // bdot equals -Ki_mag*mag_err
	matrixAdd(bdot, sv_err, bdot); // bdot now equals -Ki_mag*mag_err - Ki_sv*sv_err
	matrixScale(bdot, dt); // Multiply bdot*dt
	matrixAdd(bias, bdot, bias); // bias = bias + dt*bdot
	
	// ***** INTEGRATE DCM *****
	matrixScale(gyro_with_bias, dt);
	findRexp(gyro_with_bias, Rexp); // Rexp = findRexp(gyro_with_bias*dt)
	matrixMult(R, Rexp, new_R); // new_R = R*Rexp(gyro_with_bias*dt)
	matrixCopy(new_R, R); // R = R*Rexp(gyro_with_bias*dt)
}

/** 
 * @brief  Finds Euler angles in inertial frame from a DCM
 * @param  R: the DCM (body->inertial)
 * @param  yaw_pitch_roll: pointer to float[3] which hold yaw (0) pitch (1) and roll (2) after the function returns
 * @return None
*/
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
