/**
  ******************************************************************************
  * @file           : FeedbackControl.h
  * @brief          : Contains implementations for the feedback control states in the ACS
  ******************************************************************************
  ** 
	* As of now these functions change actuator outputs inside of the function
	* itself but that is subject to change.
	* 
	* Created by Galen Savidge. Edited 4/18/2019.
  ******************************************************************************
  */

 #ifndef FEEDBACKCONTROL_H
 #define FEEDBACKCONTROL_H
 
 #include <ACS.h>
 
 void runBdotController(ACSType* acs);
 
 void runOrientationController(ACSType* acs);
 
 void runStabilizationController(ACSType* acs);
 
 #endif
 