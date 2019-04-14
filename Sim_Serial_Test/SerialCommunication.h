/**
  ******************************************************************************
  * @file           : SerialCommunication.h
  * @brief          : Send and receive data over serial
  ******************************************************************************
  ** This module uses the libserialport library to send and receive data over
  * UART.
  * 
  * Created by Galen Savidge. Edited 4/13/2019.
  ******************************************************************************
  */

#include <libserialport.h>

typedef struct sp_port* port_t;

port_t serialInit(void);

int serialSendFloats(port_t port, float* f, unsigned int n);

int serialReceiveFloats(port_t port, float* f, unsigned int n);
