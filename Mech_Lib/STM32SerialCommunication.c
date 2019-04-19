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
#define UART_TIMEOUT 500

// Private functions
void receivePacket(UART_HandleTypeDef* huart, uint8_t* packet, unsigned int bytes) {
	HAL_StatusTypeDef status;
	do{
		status = HAL_UART_Receive(huart, packet, bytes, UART_TIMEOUT);
	} while(status != HAL_OK);
}

HAL_StatusTypeDef STM32SerialSendFloats(UART_HandleTypeDef* huart, float* f, unsigned int n) {
	// Send start packet
//	uint8_t start_packet[CONTROL_PACKET_SIZE];
//	makeControlPacket(start_packet, START);
//	HAL_UART_Transmit(huart, start_packet, CONTROL_PACKET_SIZE, UART_TIMEOUT);
	
	// Create data packet
	uint8_t data_packet[BYTES_PER_FLOAT*n];
	floatsToPacket(f, data_packet, n);

	// Send data packet
	HAL_StatusTypeDef err = HAL_UART_Transmit(huart, data_packet, BYTES_PER_FLOAT*n, UART_TIMEOUT);
	return err;

	// Add wait for ack here
}

int STM32SerialReceiveFloats(UART_HandleTypeDef* huart, float* f, unsigned int n) {
	// Wait for start packet
//	uint8_t start_packet[CONTROL_PACKET_SIZE] = {0};
//	do {
//		receivePacket(huart, start_packet, CONTROL_PACKET_SIZE);
//	} while(isControlPacket(start_packet) != START);
	
	// Receive packet
	uint8_t data_packet[BYTES_PER_FLOAT*n];
	receivePacket(huart, data_packet, BYTES_PER_FLOAT*n);
	
	// Read packet
	packetToFloats(f, data_packet, n);
	
	// Add send ack here
	return 0;
}
