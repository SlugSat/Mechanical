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

#include "STM32SerialCommunication.h"

#include <PacketProtocol.h>

#define BAUD 115200
#define UART_TIMEOUT 50

int STM32SerialSendFloats(UART_HandleTypeDef* huart, float* f, unsigned int n) {
	// Create packet
	uint8_t packet[BYTES_PER_FLOAT*n];
	floatsToPacket(f, packet, n);

	// Send packet
	HAL_StatusTypeDef err = HAL_UART_Transmit(huart, packet, BYTES_PER_FLOAT*n, UART_TIMEOUT);
	return err;

	// Add wait for ack here
}

int STM32SerialReceiveFloats(UART_HandleTypeDef* huart, float* f, unsigned int n) {
	// Receive packet
	uint8_t packet[BYTES_PER_FLOAT*n];
	HAL_StatusTypeDef status;
	do{
		status = HAL_UART_Receive(huart, packet, n*BYTES_PER_FLOAT, 10000);
	} while(status != HAL_OK);
	
	// Read packet
	packetToFloats(f, packet, n);
	
	// Add send ack here
	return 0;
}
