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

#include "main.h"

int STM32SerialSendFloats(UART_HandleTypeDef* huart, float* f, unsigned int n);

int STM32SerialReceiveFloats(UART_HandleTypeDef* huart, float* f, unsigned int n);
