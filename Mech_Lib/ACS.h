/**
  ******************************************************************************
  * @file           ACS.h
  * @brief          Contains the Attitude Control System (ACS) struct
  ******************************************************************************
    ** The ACS struct includes all the data needed to run the ACS. One struct 
	* should be created when the ACS starts. A pointer to the struct should 
	* then be passed to other ACS functions.
	* 
	* Created by Galen Savidge. Edited 5/16/2019.
  ******************************************************************************
  */

#ifndef ACS_H
#define ACS_H

/* Includes ------------------------------------------------------------------*/
#include <ACSEnums.h>
#include <Matrix.h>
#include <DigitalFilters.h>
#include <ReferenceFrames.h>
#include <main.h>
#include <stdint.h>
#include <string.h>


/* Constants -----------------------------------------------------------------*/
#define NUM_SOLAR_PANELS 6 /**< The number of faces containing solar panels (generally 5) */
#define RW_BASE_SPEED 0.0 /**< Steady state reaction wheel speed (rad/s) */
#define TR_MAXDIP 2.0 /**< Torque rod max dipole moment */


/* Datatypes -----------------------------------------------------------------*/

/** 
 * @brief A struct including all of the data needed to run the ACS
*/
typedef struct { 
	// Current time
	double julian_date; /**< Julian date in fractional days */
	float t; /**< Time in seconds since ACS startup */
	float dt; /**< Time in seconds since last ACS update */
	
	// Craft DCM
	Matrix R; /**< Craft DCM (body to ecliptic) */
	Matrix Rt; /**< Transpose of R */
	
	// Sensor vectors (body frame)
	Matrix gyro_vector; /**< Reading from the gyroscope (body frame, rad/s) */
	Matrix mag_vector; /**< Reading from the magnetometer (body frame, T)*/
	Matrix solar_vector; /**< Solar vector determined from the solar panels (body frame, normalized) */
	
	// Attitude estimation vectors
	Matrix gyro_bias; /**< Current estimated gyro bias, updated by updateAttitudeEstimate() */
	Matrix w; /**< Craft angular velocity w = gyro_vector - gyro_bias */
	float gyro_bias_dot_norm; /**< Time derivative of ||gyro bias|| */
	
	// Inertial vectors (ecliptic frame)
	Matrix sv_inertial; /**< Vector from the craft pointing at the Sun (ecliptic frame) */
	Matrix mag_inertial; /**< Earth's magnetic field from IGRF (ecliptic frame) */
	Matrix craft_inertial; /**< Craft position relative to Earth (ecliptic frame, normalized) */
	
	// Craft position in different frames
	Matrix craft_j2000; /**< Craft position relative to Earth (J2000, km) */
	float longitude; /**< In degrees east of the prime meridian, in range [0.0, 360.0) */
	float latitude; /**< In degrees north of the equator */
	float altitude; /**< In meters */
	
	// Feedback control error vectors (body frame)
	Matrix z_err; /**< Nadir pointing error vector, found by comparing zhat_B and craft_inertial (body frame) */
	Matrix n_err; /**< Error used to rotate the craft relative to the Sun */
	Matrix err; /**< Always equals the sum z_err + n_err */
	
	// Error scalars (degrees)
	float pointing_err; /**< Scalar error between zhat_B and craft_inertial (degrees) */
	float zb_sun_angle; /**< Angle between zhat_B and sv_inertial (degrees) */
	
	// Sensor hardware
	MovingAvgFilter mag_filter;
	MovingAvgFilter sv_filter;
	uint32_t sv_raw[NUM_SOLAR_PANELS]; /**< This array is used by DMA when reading ADC values */
	
	// Solar vector status
	SV_Status sun_status; /**< Current Sun status (see SV_Status) */
	
	// Satellite dynamic system
	Matrix w_rw; /**< Reaction wheel speed vector (body frame, rad/s) */ 
	Matrix J_rw; /**< Reaction wheel inertia matrix (3x3, kg*m^2) */
	Matrix J_rw_inv; /**< Inverse of J_rw (3x3, 1/(kg*m^2)) */
	Matrix A_rw; /**< Reaction wheel projection onto body axes, currently I3 */
	Matrix J_body; /**< Craft inertia matrix (3x3, kg*m^2) */
	Matrix J_body_inv; /**< Inverse of J_body (3x3, 1/(kg*m^2)) */
	
	// Actuator PWMs
	Matrix rw_PWM; /**< Reaction wheel PWMs (3x1, signed percent duty cycle) */
	uint8_t rw_brake[3]; /**< Reaction wheel brake pin values (boolean) */
	Matrix tr_PWM; /**< Torque rod PWMs (3x1, signed percent duty cycle) */
}ACSType;


/* Initialization Functions --------------------------------------------------*/

/** 
 * @brief  Initializes all the fields in the given ACS struct
 * @param  acs: a pointer to an existing Attitude Control System object
 * @return None
*/
void initializeACS(ACSType* acs);

#endif /* ACS_H */
