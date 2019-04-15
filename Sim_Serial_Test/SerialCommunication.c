/**
  ******************************************************************************
  * @file           : SerialCommunication.c
  * @brief          : Send and receive data over serial
  ******************************************************************************
  ** This module uses the libserialport library to send and receive data over
  * UART.
  * 
  * Created by Galen Savidge. Edited 4/13/2019.
  ******************************************************************************
  */

#include <SerialCommunication.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <PacketProtocol.h>

#define BAUD 115200
#define UART_TIMEOUT 50

#define STM_DESCRIPTION "STM32 STLink"

// Private functions
void receivePacket(port_t port, uint8_t* packet, unsigned int bytes) {
	int bytes_received;
	do{
		bytes_received = sp_input_waiting(port);
	} while(bytes_received < bytes);
	sp_blocking_read(port, packet, bytes_received, UART_TIMEOUT);
}

// Public functions
port_t serialInit(void) {
	// Get list of serial devices
	port_t* ports;
	enum sp_return err;
	err = sp_list_ports(&ports);
	
	port_t port;
	int i = 0;
	char* description;
	do {
		port = ports[i];
		if(port == NULL) {
			printf("STM32 board not found!\n");
			return NULL;
		}
		i++;
		description = sp_get_port_description(port);
		description[strlen(STM_DESCRIPTION)] = '\0'; // Truncate description
	} while(strcmp(description, STM_DESCRIPTION) != 0);
	
	printf("Opening port:\n");
	printf("%s | %s\n", sp_get_port_name(port), sp_get_port_description(port));
	err = sp_open(port, SP_MODE_READ_WRITE);
	if(err != 0) {
		printf("Error %d opening port!\n", err);
		return NULL;
	}

	sp_set_baudrate(port, BAUD);
	return port;
}

int serialSendFloats(port_t port, float* f, unsigned int n) {
	// Send start packet
	//uint8_t start_packet[CONTROL_PACKET_SIZE];
	//makeControlPacket(start_packet, START);
	//sp_blocking_write(port, start_packet, BYTES_PER_FLOAT*n, UART_TIMEOUT);	

	// Create data packet
	uint8_t data_packet[BYTES_PER_FLOAT*n];
	floatsToPacket(f, data_packet, n);

	// Send packet
	enum sp_return err = sp_blocking_write(port, data_packet, BYTES_PER_FLOAT*n, UART_TIMEOUT);
	return err;

	// Add wait for ack here
}

int serialReceiveFloats(port_t port, float* f, unsigned int n) {
	// Wait for start packet
	//uint8_t start_packet[CONTROL_PACKET_SIZE] = {0};
	//do {
	//	receivePacket(port, start_packet, CONTROL_PACKET_SIZE);
	//} while(isControlPacket(start_packet) != START);	

	// Receive data packet
	uint8_t data_packet[BYTES_PER_FLOAT*n];
	receivePacket(port, data_packet, BYTES_PER_FLOAT*n);

	// Read packet
	packetToFloats(f, data_packet, n);
	
	// Add send ack here
	return 0;
}
