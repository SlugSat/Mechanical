sim_serial_test : sim_serial_test.o
	gcc sim_serial_test.o /usr/local/lib/libserialport.so -o sim_serial_test

sim_serial_test.o : sim_serial_test.c	
	gcc sim_serial_test.c -o sim_serial_test.o -c --std=c99

clean :
	rm sim_serial_test sim_serial_test.o
