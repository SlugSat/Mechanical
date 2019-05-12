/**
  ******************************************************************************
  * @file           ACSStateMachine.h
  * @brief          Runs the logic for the ACS
  ******************************************************************************
  * 
	* 
	* Created by Galen Savidge. Edited 5/9/2019.
  ******************************************************************************
  */

#include <main.h>

#ifdef ENABLE_42

/** 
 * @brief  Runs the ACS in an infinite loop
 * @param  huart: a pointer to a UART handle to use to communicate with 42
 * @return None
*/
void runACS(UART_HandleTypeDef* huart);

#else

/** 
 * @brief  Runs the ACS in an infinite loop
 * @return None
*/
void runACS(void);

#endif
