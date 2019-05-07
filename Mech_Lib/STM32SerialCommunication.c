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
#include <string.h>

#define BAUD 115200
#define UART_TIMEOUT 20
#define HANDSHAKE_TIMEOUT 30

// Private functions
void receivePacket(UART_HandleTypeDef* huart, uint8_t* packet, unsigned int bytes) {
	HAL_StatusTypeDef status;
	do {
		status = HAL_UART_Receive(huart, packet, bytes, UART_TIMEOUT);
	} while(status != HAL_OK);
}


// Public functions
void STM32SerialHandshake(UART_HandleTypeDef* huart) {
	__HAL_UART_FLUSH_DRREGISTER(huart); // Flush UART registers
	
	uint8_t send[1] = {HANDSHAKE_BYTE};
	uint8_t receive[1];
	
	while (1) {
		// Send handshake byte
		HAL_UART_Transmit(huart, send, 1, UART_TIMEOUT);
		
		// Wait to receive handshake byte
		if(HAL_UART_Receive(huart, receive, 1, HANDSHAKE_TIMEOUT) == HAL_OK) {
			if(receive[0] == HANDSHAKE_BYTE) {
				__HAL_UART_FLUSH_DRREGISTER(huart); // Flush UART registers and return
				return;
			}
		}
	}
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

HAL_StatusTypeDef STM32SerialSendString(UART_HandleTypeDef* huart, char* string) {
	return HAL_UART_Transmit(huart, (uint8_t*)string, strlen(string) + 1, 2*UART_TIMEOUT);
}


int STM32SerialReceiveFloats(UART_HandleTypeDef* huart, float* f, unsigned int n) {
	// Receive packet
	uint8_t data_packet[BYTES_PER_FLOAT*n];
	//receivePacket(huart, data_packet, BYTES_PER_FLOAT*n);
	HAL_StatusTypeDef err = HAL_UART_Receive(huart, data_packet, BYTES_PER_FLOAT*n, UART_TIMEOUT);
	
	if(err == HAL_OK) {
		// Read packet
		packetToFloats(f, data_packet, n);
	}
	
	return err;
}
