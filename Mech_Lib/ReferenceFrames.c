/**
  ******************************************************************************
  * @file           : ReferenceFrames.c
  * @brief          : Contains functions to translate between reference frames.
  ******************************************************************************
  ** 
	* 
	* Created by Galen Savidge. Edited 5/2/2019.
  ******************************************************************************
  */

#include "ReferenceFrames.h"
#include <math.h>


#define J2000_TO_ECLIPTIC_ANGLE 23.5 // Degrees


// Private functions

float getGMSTDegrees(double JD) {
	double d = JD - 2451545.0; // Days since noon Jan 1, 2000
	double T = d/36525; // Centuries since noon Jan 1, 2000
	double GMST_sec = 24110.54841 + 8640184.812866 * T + 0.093104*T*T - 0.0000062*T*T*T; // Seconds in UT1
	double GMST_deg = GMST_sec/240.0; // Degrees between J2000 x-axis and the prime meridian
	return (float)GMST_deg;
}


// Public functions

void makeDCM_zyx(Matrix R, float yaw, float pitch, float roll) {
	// Angles in radians
	float y_r = yaw;
	float p_r = pitch;
	float r_r = roll;
	
	// From: https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Euler_angles_(z-y%E2%80%B2-x%E2%80%B3_intrinsic)_%E2%86%92_rotation_matrix
	
	// Row 1
	matrixSet(R, 1, 1, cos(p_r)*cos(y_r));
	matrixSet(R, 1, 2, -cos(r_r)*sin(y_r)+sin(r_r)*sin(p_r)*cos(y_r));
	matrixSet(R, 1, 3, sin(r_r)*sin(y_r)+cos(r_r)*sin(p_r)*cos(y_r));
	
	// Row 2
	matrixSet(R, 2, 1, cos(p_r)*sin(y_r));
	matrixSet(R, 2, 2, cos(r_r)*cos(y_r)+sin(r_r)*sin(p_r)*sin(y_r));
	matrixSet(R, 2, 3, -sin(r_r)*cos(y_r)+cos(r_r)*sin(p_r)*sin(y_r));
	
	// Row 3
	matrixSet(R, 3, 1, -sin(p_r));
	matrixSet(R, 3, 2, sin(r_r)*cos(p_r));
	matrixSet(R, 3, 3, cos(r_r)*cos(p_r));
}


void J2000_2_ecliptic(Matrix v_j2000, Matrix v_ecliptic) {
	static int init_run = 0;
	static Matrix xrot; // This frame conversion is a simple rotation around x
	
	if(init_run == 0) {
		xrot = newMatrix(3, 3);
		makeDCM_zyx(xrot, 0, 0, J2000_TO_ECLIPTIC_ANGLE);
		init_run = 1;
	}
	
	matrixMult(xrot, v_j2000, v_ecliptic);
}


void J2000_2_LongLatAlt(Matrix v_j2000, double JD, float* lng, float* lat, float* alt) {
	float v_norm = vectorNorm(v_j2000);
	
	// Find altitude
	*alt = v_norm - EARTH_RADIUS_KM;
	
	// Find latitude
	*lat = RAD2DEG*asin(matrixGetElement(v_j2000, 3, 1)/v_norm);
	
	// Find longitude
	*lng = RAD2DEG*acos(matrixGetElement(v_j2000, 1, 1)/v_norm) - getGMSTDegrees(JD);
}
