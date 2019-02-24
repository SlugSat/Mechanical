/**
  ******************************************************************************
  * @file           : AttitudeEstimation.h
  * @brief          : Source file for the AttitudEstimation module.
  ******************************************************************************
  ** This module contains the code to run closed loop integration of the gyro
	* using feedback from the magnetometer and solar vector.
	* 
	* Created by Galen Savidge. Edited 2/23/2019.
  ******************************************************************************
  */

#include <AttitudeEstimation.h>
#include <math.h>

// Helper functions
/** 
 * @brief  Finds the exponential Rodrigues parameter form of an angular velocity vector
 * @param  w: a 3x1 angular velocity column vector
 * @param  Rexp: an allocated 3x3 Matrix to hold the exponential Matrix
 * @return None
*/
void findRexp(Matrix w, Matrix Rexp);

double sinc(double x);

/** 
 * @brief  Allocates and initializes a 3x3 DCM Matrix
 * @param  Euler angles for the initial DCM
 * @return The new DCM Matrix
*/
Matrix initializeDCM(double yaw, double pitch, double roll) {
	Matrix r = newMatrix(3, 3);
	matrixSet(r, 1, 1, 1.0);
	matrixSet(r, 2, 2, 1.0);
	matrixSet(r, 3, 3, 1.0);
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
void integrateDCM(Matrix R, Matrix gyro, Matrix mag, Matrix sv, double dt, Matrix newR) {
	
}

// Helper function to find the sinc of a float
double sinc(double x) {
	// Taylor portion
	if(fabs(x) <= 0.5) {
		return pow(x,6)/120.0 - pow(x,2)/6.0 + 1.0;
	}  
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
	matrixSet(Rexp, 0, 0, matrixGetElement(Rexp, 0, 0) + 1);
	matrixSet(Rexp, 1, 1, matrixGetElement(Rexp, 1, 1) + 1);
	matrixSet(Rexp, 2, 2, matrixGetElement(Rexp, 2, 2) + 1);
}
