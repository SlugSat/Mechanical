/**
  ******************************************************************************
  * @file           : STM32SerialCommunication.h
  * @brief          : Send and receive data over serial
  ******************************************************************************
  ** This module handles UART communication with the flight simulation running 
	* on a Linux PC/VM.
  * 
  * Created by Galen Savidge. Edited 4/13/2019.
  ******************************************************************************
  */

#ifndef STM32SERIAL_H
#define STM32SERIAL_H

#include "main.h"

/** 
 * @brief  Blocks until completing a handshake with 42
 * @param  huart: pointer to a UART handle made by cubeMX
 * @return None
*/
void STM32SerialHandshake(UART_HandleTypeDef* huart);

/** 
 * @brief  Blocking function to send a set of n floating point numbers over serial
 * @param  huart: pointer to a UART handle made by cubeMX
 * @param  f: an array of floats of size >= n
 * @param  n: the number of floats to be sent
 * @return HAL status of the transmission
*/
HAL_StatusTypeDef STM32SerialSendFloats(UART_HandleTypeDef* huart, float* f, unsigned int n);

/** 
 * @brief  Blocking function to send a string over serial
 * @param  huart: pointer to a UART handle made by cubeMX
 * @param  string: a null terminated string
 * @return HAL status of the transmission
*/
HAL_StatusTypeDef STM32SerialSendString(UART_HandleTypeDef* huart, char* string);

/** 
 * @brief  Blocking function to receive a set of n floating point numbers from serial
 * @param  huart: pointer to a UART handle made by cubeMX
 * @param  f: an array of floats of size >= n
 * @param  n: the number of floats to receive
 * @return Always 0 for now
*/
int STM32SerialReceiveFloats(UART_HandleTypeDef* huart, float* f, unsigned int n);

#endif
