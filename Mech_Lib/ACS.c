/**
  ******************************************************************************
  * @file           : AttitudeEstimation.h
  * @brief          : Header file for the Attitude Control System (ACS).
  ******************************************************************************
  ** 
	* 
	* Created by Galen Savidge. Edited 4/21/2019.
  ******************************************************************************
  */


#include <ACS.h>
#include <AttitudeEstimation.h>
#include <STM32SerialCommunication.h>


// Reaction wheel properties
#define RW_D 8700
#define RW_R 0.0207
#define RW_H 0.005

// Body inertia matrices
#define JB_11 0.60579
#define JB_22 0.01330
#define JB_33 0.59753

#define J2000_TO_ECLIPTIC_ANGLE 23.5 // Degrees

#define RECEIVED_FLOATS 18
#define SENT_FLOATS 6


// Helper functions
Matrix makeJrw(void) {
	float m = PI*RW_R*RW_R*RW_H*RW_D; // Mass of the flywheel (kg)
	float x = 0.5*m*RW_R*RW_R;
	
	Matrix J = newMatrix(3, 3);
	matrixSet(J, 1, 1, x);
	matrixSet(J, 2, 2, x);
	matrixSet(J, 3, 3, x);
	return J;
}

Matrix makeJrwInv(void) {
	float m = PI*RW_R*RW_R*RW_H*RW_D; // Mass of the flywheel
	float x = 2.0/(m*RW_R*RW_R);
	
	Matrix J = newMatrix(3, 3);
	matrixSet(J, 1, 1, x);
	matrixSet(J, 2, 2, x);
	matrixSet(J, 3, 3, x);
	return J;
}

Matrix makeJbody(void) {
	Matrix J = newMatrix(3, 3);
	matrixSet(J, 1, 1, JB_11);
	matrixSet(J, 2, 2, JB_22);
	matrixSet(J, 3, 3, JB_33);
	return J;
}

Matrix makeJbodyInv(void) {
	Matrix J = newMatrix(3, 3);
	matrixSet(J, 1, 1, 1.0/JB_11);
	matrixSet(J, 2, 2, 1.0/JB_22);
	matrixSet(J, 3, 3, 1.0/JB_33);
	return J;
}


// Public functions
void initializeACS(ACSType* acs) {
	// Allocate Matrix structs
	acs->R = initializeDCM(0, 0, 0);
	acs->Rt = newMatrix(3, 3);
	matrixTranspose(acs->R, acs->Rt);
	
	acs->gyro_vector = newMatrix(3, 1);
	acs->mag_vector = newMatrix(3, 1);
	acs->solar_vector = newMatrix(3, 1);
	
	acs->gyro_bias = newMatrix(3, 1);
	
	acs->sv_inertial = newMatrix(3, 1);
	acs->mag_inertial = newMatrix(3, 1);
	acs->craft_inertial = newMatrix(3, 1);
	
	acs->z_err = newMatrix(3, 1);
	acs->n_err = newMatrix(3, 1);
	acs->err = newMatrix(3, 1);
	
	acs->w_rw = make3x1Vector(0, 0, 0);
	acs->J_rw = makeJrw();
	acs->J_rw_inv = makeJrwInv();
	acs->A_rw = initializeDCM(0, 0, 0); // I3 (identity matrix)
	acs->J_body = makeJbody();
	acs->J_body_inv = makeJbodyInv();
	
	acs->rw_PWM = newMatrix(3, 1);
	acs->tr_PWM = newMatrix(3, 1);
}


void initializeACSSerial(ACSType* acs, UART_HandleTypeDef* huart) {
	acs->huart = huart;
}


void readSensorsFromSerial(ACSType* acs) {
	static int init_run = 0;
	static Matrix c_i_j2000;
	
	if(init_run == 0) {
		c_i_j2000 = newMatrix(3, 1);
		init_run = 1;
	}
	
	// Read floats from UART
	float sensor_data[RECEIVED_FLOATS];
	STM32SerialReceiveFloats(acs->huart, sensor_data, RECEIVED_FLOATS);
	
	// Get sensor vectors
	vectorCopyArray(acs->mag_vector, sensor_data, 3);
	vectorCopyArray(acs->gyro_vector, sensor_data + 3, 3);
	vectorCopyArray(acs->solar_vector, sensor_data + 6, 3);
	
	// Check for invalid solar vector
	if(vectorNorm(acs->solar_vector) == 0) {
		acs->sun_status = SV_DARK;
		vectorSetXYZ(acs->solar_vector, 1, 0, 0); // Set to x+ to avoid divide by 0 errors
	}
	else {
		acs->sun_status = SV_FOUND;
	}
	
	// Get position in Ecliptic ECI frame
	vectorCopyArray(c_i_j2000, sensor_data + 9, 3);
	J2000_2_ecliptic(c_i_j2000, acs->craft_inertial);
	vectorCopyArray(acs->w_rw, sensor_data + 12, 3);
	
	// Get current time
	acs->julian_date = (double)sensor_data[15] + (double)sensor_data[16]; // Reconstruct Julian date
	float new_t = sensor_data[17];
	acs->dt = new_t - acs->t;
	acs->t = new_t;
}


void sendActuatorsToSerial(ACSType* acs) {
	// Packet: {rw_x, rw_y, rw_z, tr_x, tr_y, tr_z}
	float actuator_data[SENT_FLOATS] =
		{ matrixGetElement(acs->rw_PWM, 1, 1), matrixGetElement(acs->rw_PWM, 2, 1), matrixGetElement(acs->rw_PWM, 3, 1),
			matrixGetElement(acs->tr_PWM, 1, 1), matrixGetElement(acs->tr_PWM, 2, 1), matrixGetElement(acs->tr_PWM, 3, 1) };
	
		STM32SerialSendFloats(acs->huart, actuator_data, SENT_FLOATS);
}


void J2000_2_ecliptic(Matrix v_j2000, Matrix v_ecliptic) {
	static int init_run = 0;
	static Matrix xrot; // This frame conversion is a simple rotation around x
	
	if(init_run == 0) {
		xrot = initializeDCM(0, 0, J2000_TO_ECLIPTIC_ANGLE);
		init_run = 1;
	}
	
	matrixMult(xrot, v_j2000, v_ecliptic);
}
