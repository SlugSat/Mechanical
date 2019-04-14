#include <stdio.h>
#include <libserialport.h>
#include <PacketProtocol.h>

#define BAUD 115200

int main(int argc, char *argv[]) {
	// Packet unit tests
	int num_floats = 4;
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
	printf("\r\n\r\n");
	
	/* struct sp_port** ports;
	enum sp_return err;
	err = sp_list_ports(&ports);

	// List serial ports
	printf("Serial ports:\n");
	int i;
	for(i = 0;ports[i] != NULL;i++) {
		printf("%s | %s\n", sp_get_port_name(ports[i]), sp_get_port_description(ports[i]));
	}

	// Exit if the port list is empty
	if(i == 0) {
		printf("No serial devices found, exiting...\n");
		return(0);
	}

	// Open last item on the list
	struct sp_port* port = ports[0];
	printf("Opening port:\n");
	printf("%s | %s\n", sp_get_port_name(port), sp_get_port_description(port));
	err = sp_open(port, SP_MODE_READ_WRITE);
	if(err != 0) {
		printf("Error %d opening port!\n", err);
		return(1);
	}

	sp_set_baudrate(port, BAUD);

	char buffer[1024];
	for(i=0;1;i++) {
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
	}*/
}
