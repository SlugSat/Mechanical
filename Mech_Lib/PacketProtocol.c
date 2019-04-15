/**
  ******************************************************************************
  * @file           : PacketProtocol.h
  * @brief          : Source file for the PacketProtocol module.
  ******************************************************************************
  ** Use this module to create packets to send to and from the flat-sat over
	* serial.
	* 
	* Created by Galen Savidge. Edited 4/13/2019.
  ******************************************************************************
  */

#include <PacketProtocol.h>

#define get_byte_n(x, n) (((x) & (0xFF << ((n)*8))) >> ((n)*8))

// Writes n floats to a packet of size n*4 bytes
void floatsToPacket(float* f, uint8_t* p, unsigned int n) {
	unsigned int p_i = 0; // Packet index
	uint32_t* f_bytes = (uint32_t*)f;
	for(unsigned int f_i = 0;f_i < n;f_i++) { // Loop over floats	
		for(unsigned int b_i = 0;b_i < BYTES_PER_FLOAT;b_i++) { // Loop over bytes in float
			p[p_i] = get_byte_n(f_bytes[f_i], b_i);
			p_i++;
		}
	}
}

// Reads n floats from a packet of size n*4 bytes
void packetToFloats(float* f, uint8_t* p, unsigned int n) {
	unsigned int p_i = 0; // Packet index
	uint32_t* f_bytes = (uint32_t*)f;
	for(unsigned int f_i = 0;f_i < n;f_i++) { // Loop over floats	
		f_bytes[f_i] = 0x0; // Clear the bits in float f_i
		for(unsigned int b_i = 0;b_i < BYTES_PER_FLOAT;b_i++) { // Loop over bytes in float
			f_bytes[f_i] |= p[p_i]<<(b_i*8);
			p_i++;
		}
	}
}

// Makes a 4-byte control packet
void makeControlPacket(uint8_t* p, ControlPacketType type) {
	p[0] = (uint8_t)type;
	for(int i = 1;i < 4;i++) {
		p[i] = 0xFF;
	}
}

// Checks if 4-byte packet p is a control packet
ControlPacketType isControlPacket(uint8_t* p) {
	for(int i = 1;i < 4;i++) {
		if(p[i] != 0xFF) {
			return NONE;
		}
	}
	return (ControlPacketType)p[0];
}
