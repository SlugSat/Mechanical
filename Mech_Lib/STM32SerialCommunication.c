/*
  ******************************************************************************
  * @file           STM32SerialCommunication.c
  * @brief          A library to send and receive data from 42
  ******************************************************************************
  * This module handles UART communication with 42 SlugSat (our customized 
  * flight simulation).
  * 
  * Created by Galen Savidge. Edited 5/12/2019.
  ******************************************************************************
  */

#include "STM32SerialCommunication.h"

#include <PacketProtocol.h>
#include <string.h>

#define BAUD 115200
#define UART_TIMEOUT 20
#define HANDSHAKE_TIMEOUT 20

#define RECEIVED_FLOATS 18
#define SENT_FLOATS 6


// Module level variables
static UART_HandleTypeDef* huart;
static char printstring[1000];


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

void initializeACSSerial(UART_HandleTypeDef* uart) {
	huart = uart;
}

void syncWith42(ACSType* acs) {
	STM32SerialHandshake(huart);
	readSensorsFromSerial(acs);
	sendActuatorsToSerial(acs);
	STM32SerialSendString(huart, printstring);
	printstring[0] = '\0'; // Clear printstring
}

void readSensorsFromSerial(ACSType* acs) {
	// Read floats from UART
	float sensor_data[RECEIVED_FLOATS];
	if(STM32SerialReceiveFloats(huart, sensor_data, RECEIVED_FLOATS) != HAL_OK) {
		return;
	}
	
	// Get sensor vectors
	vectorCopyArray(acs->mag_vector, sensor_data, 3);
	matrixScale(acs->mag_vector, 1e-6); // Convert uT to T
	vectorCopyArray(acs->gyro_vector, sensor_data + 3, 3);
	vectorCopyArray(acs->solar_vector, sensor_data + 6, 3);

	// Check for invalid solar vector
	if(vectorNorm(acs->solar_vector) == 0) {
		acs->sun_status = SV_DARK;
	}
	//else if(matrixGetElement(acs->solar_vector, 3, 1) == 0) {
	//	acs->sun_status = SV_NOTFOUND;
	//}
	else {
		acs->sun_status = SV_FOUND;
	}
	
	// Get position in Ecliptic ECI frame
	vectorCopyArray(acs->craft_j2000, sensor_data + 9, 3);
	J2000_2_ecliptic(acs->craft_j2000, acs->craft_inertial);
	float c_I_norm = vectorNorm(acs->craft_inertial);
	if(c_I_norm != 0) {
		matrixScale(acs->craft_inertial, 1.0/c_I_norm); // Normalize c_I
	}
	
	// Get reaction wheel speeds
	vectorCopyArray(acs->w_rw, sensor_data + 12, 3);
	
	// Get current time
	acs->julian_date = (double)sensor_data[15] + (double)sensor_data[16]; // Reconstruct Julian date
	float new_t = sensor_data[17];
	acs->dt = new_t - acs->t;
	acs->t = new_t;
}


void sendActuatorsToSerial(ACSType* acs) {
	// Packet: {rw_x, rw_y, rw_z, tr_x, tr_y, tr_z}
	float actuator_data[SENT_FLOATS] =
		{ matrixGetElement(acs->rw_PWM, 1, 1), matrixGetElement(acs->rw_PWM, 2, 1), matrixGetElement(acs->rw_PWM, 3, 1),
			matrixGetElement(acs->tr_PWM, 1, 1), matrixGetElement(acs->tr_PWM, 2, 1), matrixGetElement(acs->tr_PWM, 3, 1) };
	
		STM32SerialSendFloats(huart, actuator_data, SENT_FLOATS);
}

void printTo42(char* str) {
	strcat(printstring, str);
}
