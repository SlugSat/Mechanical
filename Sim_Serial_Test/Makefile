INCLUDE	= -I../Mech_Lib/

sim_serial_test : sim_serial_test.o PacketProtocol.o
	gcc sim_serial_test.o PacketProtocol.o -o sim_serial_test /usr/local/lib/libserialport.so

sim_serial_test.o : sim_serial_test.c	
	gcc sim_serial_test.c -o sim_serial_test.o -c --std=c99 $(INCLUDE)

PacketProtocol.o : PacketProtocol.c PacketProtocol.h	
	gcc PacketProtocol.c -o PacketProtocol.o -c --std=c99 $(INCLUDE)

clean :
	rm sim_serial_test *.o
