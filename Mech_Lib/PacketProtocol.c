

#include <PacketProtocol.h>

#define BYTES_PER_FLOAT 4

#define get_byte_n(x, n) (((x) & (0xFF<<((n)*8))) >> ((n)*8))

// Writes n floats to a packet of size n*4 bytes
void floatsToPacket(float* f, char* p, unsigned int n) {
	unsigned int p_i = 0; // Packet index
	uint32_t* f_bits = (uint32_t*)f;
	for(unsigned int f_i = 0;f_i < n;f_i++) { // Loop over floats	
		for(unsigned int b_i = 0;b_i < BYTES_PER_FLOAT;b_i++) { // Loop over bytes in float
			p[p_i] = get_byte_n(f_bits[f_i], b_i);
			p_i++;
		}
	}
}

// Reads n floats from a packet of size n*4 bytes
void packetToFloats(float* f, char* p, unsigned int n) {
	unsigned int p_i = 0; // Packet index
	uint32_t* f_bits = (uint32_t*)f;
	for(unsigned int f_i = 0;f_i < n;f_i++) { // Loop over floats	
		f_bits[f_i] = 0x0; // Clear the bits in float f_i
		for(unsigned int b_i = 0;b_i < BYTES_PER_FLOAT;b_i++) { // Loop over bytes in float
			f_bits[f_i] &= p[p_i]<<(b_i*8);
			p_i++;
		}
	}
}
