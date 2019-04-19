/**
  ******************************************************************************
  * @file           : AttitudeEstimation.c
  * @brief          : Source file for the AttitudEstimation module.
  ******************************************************************************
  ** This module contains the code to run closed loop integration of the gyro
	* using feedback from the magnetometer and solar vector.
	* 
	* See the header file for function descriptions.
	* 
	* Created by Galen Savidge. Edited 3/6/2019.
  ******************************************************************************
  */

// LIBRARIES
#include <AttitudeEstimation.h>
#include <BNO055_IMU.h>
#include <STM32SerialCommunication.h>
#include <SolarVectors.h>
#include <math.h>


// CONSTANTS
#define PI 3.141592654

#define KP_MAG_BASE 1.0
#define KI_MAG_BASE 0.3
#define KP_SV_BASE 1.0
#define KI_SV_BASE 0.3

#define MAG_HIST_LENGTH 10
#define SV_HIST_LENGTH 10

#define SENSOR_READ_DELAY_MS 50

// HELPER FUNCTIONS
/** 
 * @brief  Finds the exponential Rodrigues parameter form of an angular velocity vector
 * @param  w: a 3x1 angular velocity column vector
 * @param  Rexp: an allocated 3x3 Matrix to hold the exponential Matrix
 * @return None
*/
void findRexp(Matrix w, Matrix Rexp);

/**
 * @brief	Sinc function that handles the case where x ~= 0
*/
float	sinc(float x);


// PUBLIC FUNCTIONS

void initializeSensors(ACSType* acs, I2C_HandleTypeDef* hi2c, ADC_HandleTypeDef* hadc) {
	// Sensor hardware
	IMU_init(hi2c, OPERATION_MODE_MAGGYRO);
	acs->hi2c = hi2c;
	HAL_ADC_Start_DMA(hadc, acs->sv_raw, NUM_SOLAR_PANELS);
	
	// Sensor moving average filters
	acs->mag_filter = newMovingAvgFilter(3, MAG_HIST_LENGTH);
	acs->sv_filter = newMovingAvgFilter(NUM_SOLAR_PANELS, SV_HIST_LENGTH);
	
	// Read sensors until moving average filters are full
	uint8_t sensor_iterations = SV_HIST_LENGTH > MAG_HIST_LENGTH ? SV_HIST_LENGTH : MAG_HIST_LENGTH;
	for(int i = 0;i < sensor_iterations;i++) {
		readSensors(acs);
		HAL_Delay(SENSOR_READ_DELAY_MS);
	}
}


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

void readSensors(ACSType* acs) {
	static float gyro_data[3], mag_data[3], sv_data[NUM_SOLAR_PANELS];
	
	// Read gyro and transform into a column vector Matrix
	get_gyr_data(acs->hi2c, gyro_data);
	vectorCopyArray(acs->gyro_vector, gyro_data, 3);
	
	// Read magnetometer, iterate moving average filter, transform into a vector Matrix
	get_mag_data_corrected(acs->hi2c, mag_data);
	runMovingAvgFilter(acs->mag_filter, mag_data);
	vectorCopyArray(acs->mag_vector, mag_data, 3);
	
	// Read solar vector
	for(int i = 0;i < NUM_SOLAR_PANELS;i++) {
		sv_data[i] = ADC_TO_VOLTS(acs->sv_raw[i]);
	}
	runMovingAvgFilter(acs->sv_filter, sv_data);
	acs->sun_status = findSolarVector(sv_data, NUM_SOLAR_PANELS, acs->solar_vector);
}


void updateAttitudeEstimate(ACSType* acs, float dt) {
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
	
	integrateDCM(acs, Kp_mag, Ki_mag, Kp_sv, Ki_sv, dt);
}


void integrateDCM(ACSType* acs, float Kp_mag, float Ki_mag, float Kp_sv, float Ki_sv, float dt) {
			
	static char init_run = 0;
	static Matrix Rt; // Transpose of the DCM
	static Matrix mag_i_body, sv_i_body; // Inertial sensor vectors translated to body
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
		// printf("EULER ERROR: DIVIDE BY 0");
		while(1);
	}
	
	matrixScale(acs->mag_vector, 1.0/norm_mag);
	matrixScale(acs->solar_vector, 1.0/norm_sv);
	matrixScale(acs->mag_inertial, 1.0/norm_mi);
	matrixScale(acs->sv_inertial, 1.0/norm_svi);
	
	// ***** TRANSPOSE DCM *****
	matrixTranspose(acs->R, Rt);
	
	// ***** FIND ERROR FROM MAG *****
	matrixMult(Rt, acs->mag_inertial, mag_i_body); // Translate mag to body
	vectorCrossProduct(acs->mag_vector, mag_i_body, mag_err);
	matrixCopy(mag_err, merr_x_Kp);
	matrixScale(merr_x_Kp, Kp_mag); // Kp_mag * mag_err
	
	// ***** FIND ERROR FROM SOLAR VECTOR *****
	matrixMult(Rt, acs->sv_inertial, sv_i_body); // Translate mag to body
	vectorCrossProduct(acs->solar_vector, sv_i_body, sv_err); // Cross product
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
	matrixScale(bdot, dt); // Multiply bdot*dt
	matrixAdd(acs->gyro_bias, bdot, acs->gyro_bias); // bias = bias + dt*bdot
	
	// ***** INTEGRATE DCM *****
	matrixScale(gyro_with_bias, dt);
	findRexp(gyro_with_bias, Rexp); // Rexp = findRexp(gyro_with_bias*dt)
	matrixMult(acs->R, Rexp, new_R); // new_R = R*Rexp(gyro_with_bias*dt)
	matrixCopy(new_R, acs->R); // R = R*Rexp(gyro_with_bias*dt)
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
