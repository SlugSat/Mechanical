#include <stdio.h>
#include <libserialport.h>
#include <PacketProtocol.h>
#include <SerialCommunication.h>

#define BAUD 115200

int main(int argc, char *argv[]) {
	// Packet unit tests
	/*int num_floats = 4;
	float send[] = {(float)3.1415926535, 0.0, -0.0, 0.0001};
	uint8_t packet[num_floats*4];
	float receive[num_floats];
	
	floatsToPacket(send, packet, num_floats);
	packetToFloats(receive, packet, num_floats);

	printf("Sent floats:\r\n");
	for(int i = 0;i < num_floats;i++) {
		if(i != 0) {
			printf("|");
		}
		printf("%4.6f", send[i]);
	}
	printf("\r\n\r\n");

	printf("Packet:\r\n");
	for(int i = 0;i < 4*num_floats;i++) {
		if(i != 0) {
			printf("|");
		}
		printf("0x%02x", packet[i]);
	}
	printf("\r\n\r\n");

	printf("Received floats:\r\n");
	for(int i = 0;i < num_floats;i++) {
		if(i != 0) {
			printf("|");
		}
		printf("%4.6f", receive[i]);
	}
	printf("\r\n\r\n");*/
	

	port_t port = serialInit();

	char buffer[1024];
	for(int i=0;1;i++) {
		// Send bytes
//		char transmit[50];
//		sprintf(transmit, "Message %d\r\n", i);
//		sp_blocking_write(port, transmit);

		// Receive bytes
		int bytes_received;
		do{
			bytes_received = sp_input_waiting(port);
		} while(bytes_received == 0);
		sp_blocking_read(port, buffer, bytes_received, 50);
		buffer[bytes_received] = '\0';
		printf("%s", buffer);
	}
}
