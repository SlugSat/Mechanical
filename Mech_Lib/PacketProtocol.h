/**
  ******************************************************************************
  * @file           PacketProtocol.h
  * @brief          Makes packets to send to 42 and decodes packets from 42
  ******************************************************************************
	* Created by Galen Savidge. Edited 5/12/2019.
  ******************************************************************************
  */

#ifndef PACKETPROTOCOL_H
#define PACKETPROTOCOL_H

/* Dependencies ------------------------------------------------------------*/
#include <stdint.h>


/* Public constants --------------------------------------------------------*/
#define BYTES_PER_FLOAT 4
#define CONTROL_PACKET_SIZE BYTES_PER_FLOAT
#define HANDSHAKE_BYTE 0xA5


/* Public functions prototypes ---------------------------------------------*/

/** 
 * @brief  Writes n floats to a packet of size n*4 bytes
 * @param  f: array of floats to be sent
 * @param  p: byte array of size n*4 to contain the packet
 * @param  n: number of floats in f
 * @return p is a packet containing n values from f
*/
void floatsToPacket(float* f, uint8_t* p, unsigned int n);

/** 
 * @brief  Reads n floats from a packet of size n*4 bytes
 * @param  f: array to hold received floats
 * @param  p: received packet (must have size n*4)
 * @param  n: size of f
 * @return f contains n values from p
*/
void packetToFloats(float* f, uint8_t* p, unsigned int n);

#endif
