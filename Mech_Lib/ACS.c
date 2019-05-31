/*
  ******************************************************************************
  * @file           ACS.c
  * @brief          Contains the Attitude Control System (ACS) struct
  ******************************************************************************
  ** The ACS struct includes all the data needed to run the ACS. One struct 
  * should be created when the ACS starts. A pointer to the struct should 
  * then be passed to other ACS functions as appropriate.
  * 
  * Created by Galen Savidge. Edited 5/11/2019.
  ******************************************************************************
  */


#include <ACS.h>
#include <AttitudeEstimation.h>
#include <ReferenceFrames.h>


// Reaction wheel properties
#define RW_D 8700
#define RW_R 0.010
#define RW_H 0.010

// Body inertia matrices
#define JB_11 0.01151603
#define JB_22 0.10225145
#define JB_33 0.09528751


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
	acs->t = 0;
	
	// Allocate Matrix structs
	acs->R = initializeDCM(0, 0, 0);
	acs->Rt = newMatrix(3, 3);
	matrixTranspose(acs->R, acs->Rt);
	
	acs->gyro_vector = newMatrix(3, 1);
	acs->mag_vector = newMatrix(3, 1);
	acs->solar_vector = newMatrix(3, 1);
	
	acs->gyro_bias = make3x1Vector(0, 0, 0);
	acs->w = newMatrix(3, 1);
	
	acs->sv_inertial = newMatrix(3, 1);
	acs->mag_inertial = newMatrix(3, 1);
	acs->craft_inertial = newMatrix(3, 1);
	acs->craft_j2000 = newMatrix(3, 1);
	
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
	for(int i = 0;i < 3;i++) {
		acs->rw_brake[i] = 0;
	}
	acs->tr_PWM = newMatrix(3, 1);
}
