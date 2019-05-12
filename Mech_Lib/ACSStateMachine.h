/**
  ******************************************************************************
  * @file           ACSStateMachine.h
  * @brief          Runs the logic for the ACS
  ******************************************************************************
  * 
	* 
	* Created by Galen Savidge. Edited 5/11/2019.
  ******************************************************************************
  */

#ifndef ACSSTATEMACHINE_H
#define ACSSTATEMACHINE_H


#include <main.h>


#ifdef ENABLE_42
/**
 * @brief  Sets the UART handle used to communicate with 42
 * @param  uart: pointer to a UART handle from main.c
 * @return None
*/
void setUartHandle(UART_HandleTypeDef* uart);
#endif
#ifdef ENABLE_FRAM
/**
 * @brief  Sets the SPI handle used to communicate with FRAM
 * @param  spi: pointer to a SPI handle from main.c
 * @return None
*/
void setSpiHandle(SPI_HandleTypeDef* spi);
#endif


/**
 * @brief  Runs the ACS in an infinite loop
 * @return None
*/
void runACS(void);

#endif
