/**
  ******************************************************************************
  * @file           ACSEnums.h
  * @brief          
  ******************************************************************************
    ** 
	* 
	* Created by Galen Savidge. Edited 5/16/2019.
  ******************************************************************************
  */

#ifndef ACSENUMS_H
#define ACSENUMS_H


/* Datatypes -----------------------------------------------------------------*/

/**
 * @brief An enum for the state of the craft's solar panels and calculated solar vector
 */
typedef enum {
	SV_FOUND = 0, /**< Solar vector is reliable */
	SV_NOTFOUND, /**< Craft is in the sun but solar vector is unreliable */
	SV_DARK /**< Craft is in eclipse */
}SV_Status;

/**
 * @brief An enum for the states of the ACS
 */
typedef enum {
	DEFAULT = 0,
	WAIT_FOR_ENABLE,
	DETUMBLE,
	WAIT_FOR_ATTITUDE,
	REORIENT,
	STABILIZE_NO_SUN,
	STABILIZE
}ACSState;


#endif /* ACSENUMS_H */
