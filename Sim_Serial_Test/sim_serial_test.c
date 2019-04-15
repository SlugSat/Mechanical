#include <stdio.h>
#include <libserialport.h>
#include <PacketProtocol.h>
#include <SerialCommunication.h>

#define BAUD 115200

int main(int argc, char *argv[]) {
	// Packet unit tests
	unsigned char packet[CONTROL_PACKET_SIZE];
	makeControlPacket(packet, START);
	ControlPacketType t = isControlPacket(packet);
	printf("Start packet type (should be 1): %d\n\n", t);
	
	int num_floats = 4;
	float send[] = {(float)3.1415926535, 0.0, -0.0, 0.0001};
	float receive[num_floats];
	
	port_t port = serialInit();
	serialSendFloats(port, send, num_floats);
	serialReceiveFloats(port, receive, num_floats);

	printf("Sent floats:\r\n");
	for(int i = 0;i < num_floats;i++) {
		if(i != 0) {
			printf("|");
		}
		printf("%4.6f", send[i]);
	}
	printf("\r\n\r\n");

	printf("Received floats:\r\n");
	for(int i = 0;i < num_floats;i++) {
		if(i != 0) {
			printf("|");
		}
		printf("%4.6f", receive[i]);
	}
	printf("\r\n\r\n");
}
