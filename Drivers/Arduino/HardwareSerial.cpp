/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include "HardwareSerial.h"


// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
#define SERIAL_BUFFER_SIZE 64

struct ring_buffer{
    unsigned char buffer[SERIAL_BUFFER_SIZE];
    volatile unsigned int head;
    volatile unsigned int tail;
};

ring_buffer rx_buffer = { { 0 }, 0, 0};

inline void store_char(unsigned char c, ring_buffer *buffer)
{
    unsigned int i = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != buffer->tail) {
        buffer->buffer[buffer->head] = c;
        buffer->head = i;
    }
}

unsigned long uartCallback(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){
    unsigned char c;

    if(ulMsgParam & xUART_INT_RX){
        if(!(xUARTRxErrorGet(sUART_BASE) & xUART_RXERROR_FRAMING)){ //Èç¹ûÎÞÖ¡´íÎó
        	c = xHWREG(sUART_BASE + USART_DR);
            store_char(c, &rx_buffer);
        } else {
            c = xUARTCharGetNonBlocking(sUART_BASE); //c = xHWREG(sUART_BASE + USART_DR);
        } 
    }
    return 0;
}

// Constructors ////////////////////////////////////////////////////////////////
HardwareSerial::HardwareSerial(ring_buffer *rx_buffer, uint32_t UARTBase)
{
    _rx_buffer = rx_buffer;
    _rx_buffer->head = _rx_buffer->tail = 0;
    this->serialPort = UARTBase;
}

// Public Methods //////////////////////////////////////////////////////////////
void HardwareSerial::begin(unsigned long baud)
{
    xSysCtlPeripheralEnable2(serialPort);
    xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD0));

    sPinTypeUART(serialPort);
    
    xUARTConfigSet(serialPort, baud, (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_NONE));
    xUARTEnable(serialPort, (xUART_BLOCK_UART | xUART_BLOCK_TX | xUART_BLOCK_RX));
    
    xUARTIntCallbackInit(serialPort, uartCallback);  
    xUARTIntEnable(serialPort, xUART_INT_RX);
    xIntEnable(xSysCtlPeripheraIntNumGet(serialPort));

    transmiting = false;
}

void HardwareSerial::begin(unsigned long baud, unsigned long config)
{
    xSysCtlPeripheralEnable2(serialPort);
    xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD0)); 
    sPinTypeUART(serialPort);
    xUARTConfigSet(serialPort, baud, config);
    xUARTEnable(serialPort, (xUART_BLOCK_UART | xUART_BLOCK_TX | xUART_BLOCK_RX));
    
    xUARTIntCallbackInit(serialPort, uartCallback);  
    xUARTIntEnable(serialPort, xUART_INT_RX);
    xIntEnable(xSysCtlPeripheraIntNumGet(serialPort));
}

void HardwareSerial::end()
{ 
    //xUARTIntDisable(serialPort, xUART_INT_RX);	//TEST
    xUARTDisable(serialPort, (xUART_BLOCK_UART | xUART_BLOCK_TX | xUART_BLOCK_RX));
    xSysCtlPeripheralDisable2(serialPort);
    // clear any received data
    _rx_buffer->head = _rx_buffer->tail;
}

int HardwareSerial::available(void)
{
    return (unsigned int)(SERIAL_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) % SERIAL_BUFFER_SIZE;
}

int HardwareSerial::peek(void)
{
    if (_rx_buffer->head == _rx_buffer->tail) {
        return -1;
    } else {
        return _rx_buffer->buffer[_rx_buffer->tail];
    }
}

int HardwareSerial::read(void)
{
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rx_buffer->head == _rx_buffer->tail) {
        return -1;
    } else {
        unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
        _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % SERIAL_BUFFER_SIZE;
        return c;
    }
}

void HardwareSerial::flush()
{
    // UDR is kept full while the buffer is not empty, so TXC triggers when EMPTY && SENT
	while (transmiting && xUARTBusy(serialPort));
    transmiting = false;
}

size_t HardwareSerial::write(uint8_t c)
{
    xUARTCharPut(serialPort, c);
    return 1;
}

HardwareSerial::operator bool() {
    return true;
}

// Preinstantiate Objects //////////////////////////////////////////////////////

HardwareSerial Serial(&rx_buffer, sUART_BASE);
