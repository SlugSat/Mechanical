/**
  ******************************************************************************
  * @file           : PacketProtocol.h
  * @brief          : Header for the PacketProtocol module.
  ******************************************************************************
  ** Use this module to create packets to send to and from the flat-sat over
	* serial.
	* 
	* Created by Galen Savidge. Edited 3/6/2019.
  ******************************************************************************
  */

#include <stdint.h>

#define BYTES_PER_FLOAT 4

/* Public functions prototypes ---------------------------------------------*/

/** 
 * @brief  Writes n floats to a packet of size n*4 bytes
 * @param  f: array of floats to be sent
 * @param  p: byte array of size n*4 to contain the packet
 * @param  n: number of floats in f
 * @return None
*/
void floatsToPacket(float* f, uint8_t* p, unsigned int n);

/** 
 * @brief  Reads n floats from a packet of size n*4 bytes
 * @param  f: array to hold received floats
 * @param  p: received packet (must have size n*4)
 * @param  n: size of f
 * @return None
*/
void packetToFloats(float* f, uint8_t* p, unsigned int n);

/** 
 * @brief  Makes an acknowledgement packet in packet array p
 * @param  p: byte array of size 4 to contain the packet
 * @return None
*/
void makeAckPacket(uint8_t* p);

/** 
 * @brief  Checks if p contains an acknowledgement packet
 * @param  p: received packet of size 4
 * @return Nonzero if p is an ack packet, zero otherwise
*/
int isAckPacket(uint8_t* p);
