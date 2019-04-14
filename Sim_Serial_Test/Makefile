INCLUDE	= -I.

sim_serial_test : sim_serial_test.o PacketProtocol.o SerialCommunication.o
	gcc sim_serial_test.o PacketProtocol.o SerialCommunication.o /usr/local/lib/libserialport.so -o sim_serial_test 

sim_serial_test.o : sim_serial_test.c	
	gcc sim_serial_test.c -o sim_serial_test.o -c --std=c99 $(INCLUDE)

PacketProtocol.o : PacketProtocol.c PacketProtocol.h	
	gcc PacketProtocol.c -o PacketProtocol.o -c --std=c99 $(INCLUDE)

SerialCommunication.o : SerialCommunication.c SerialCommunication.h	
	gcc SerialCommunication.c -o SerialCommunication.o -c --std=c99 $(INCLUDE)

clean :
	rm sim_serial_test *.o
