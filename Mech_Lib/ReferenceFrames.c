/*
  ******************************************************************************
  * @file           ReferenceFrames.c
  * @brief          Contains functions to translate between reference frames.
  ******************************************************************************
  * Created by Galen Savidge. Edited 5/12/2019.
  ******************************************************************************
  */

#include "ReferenceFrames.h"
#include <math.h>


#define J2000_TO_ECLIPTIC_ANGLE -0.4101524 // -23.5 degrees in radians


// Private functions

float getGMSTDegrees(double JD) {
	double d = JD - 2451545.0; // Days since noon Jan 1, 2000
	double GMST_hr = 18.697374558 + 24.06570982441908*d;
	double GMST_deg = 15.0*GMST_hr;
	while(GMST_deg < 0) {
		GMST_deg += 360.0;
	}
	while(GMST_deg >= 360.0) {
		GMST_deg -= 360.0;
	}
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
	float v_xy_norm = sqrt(pow(matrixGetElement(v_j2000, 1, 1), 2) + pow(matrixGetElement(v_j2000, 2, 1), 2)); // Norm of proj of v onto xy plane
	
	// Find altitude
	*alt = v_norm - EARTH_RADIUS_KM*1000;
	
	// Find latitude
	*lat = RAD2DEG*asin(matrixGetElement(v_j2000, 3, 1)/v_norm);
	
	// Find longitude
	float alpha_deg = RAD2DEG*(float)acos(matrixGetElement(v_j2000, 1, 1)/v_xy_norm); // Angle from J2000 x-axis to craft J2000 position projection onto equatorial plane
	if(matrixGetElement(v_j2000, 2, 1) < 0) {
		alpha_deg = 360.0 - alpha_deg; // Make sure the angle is in the correct quadrant
	}
	*lng = alpha_deg - getGMSTDegrees(JD);
	while(*lng < 0) {
		*lng += 360.0;
	}
	while(*lng >= 360.0) {
		*lng -= 360.0;
	}
}


void NED_2_J2000(Matrix v_ned, Matrix c_j2000, double JD, Matrix v_j2000) {
	static int init_run = 0;
	static Matrix R, Rxv;
	
	if(init_run == 0) {
		R = newMatrix(3, 3);
		Rxv = newMatrix(3, 1);
		init_run = 1;
	}
	
	// Find rotation angles
	float c_norm = vectorNorm(c_j2000);
	float c_xy_norm = sqrt(pow(matrixGetElement(c_j2000, 1, 1), 2) + pow(matrixGetElement(c_j2000, 2, 1), 2)); // Norm of proj of v onto xy plane
	float theta = -asin(matrixGetElement(c_j2000, 3, 1)/c_norm);
	float alpha = (float)acos(matrixGetElement(c_j2000, 1, 1)/c_xy_norm); // Angle from J2000 x-axis to craft J2000 position projection onto equatorial plane
	if(matrixGetElement(c_j2000, 2, 1) < 0) {
		alpha = 2*PI - alpha; // Make sure the angle is in the correct quadrant
	}
	
	// Make rotation matrix
	// Row 1
	matrixSet(R, 1, 1, cos(theta));
	matrixSet(R, 1, 2, 0);
	matrixSet(R, 1, 3, sin(theta));
	
	// Row 2
	matrixSet(R, 2, 1, sin(alpha)*sin(theta));
	matrixSet(R, 2, 2, cos(alpha)); 
	matrixSet(R, 2, 3, -cos(theta)*sin(alpha)); 
	
	// Row 3
	matrixSet(R, 3, 1, -cos(alpha)*sin(theta));
	matrixSet(R, 3, 2, sin(alpha)); 
	matrixSet(R, 3, 3, cos(alpha)*cos(theta));
	
	// Multiply by rotation matrix
	matrixMult(R, v_ned, Rxv);
	
	// Assemble J2000 vector
	matrixSet(v_j2000, 1, 1, -matrixGetElement(Rxv, 3, 1)); // x = -down
	matrixSet(v_j2000, 2, 1, matrixGetElement(Rxv, 2, 1)); // y = east
	matrixSet(v_j2000, 3, 1, matrixGetElement(Rxv, 1, 1)); // z = north
}
