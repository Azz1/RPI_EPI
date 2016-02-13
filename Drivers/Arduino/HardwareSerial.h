/*
  HardwareSerial.h - Hardware serial library for Wiring
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

  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
*/

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>
#include "Stream.h"

struct ring_buffer;

class HardwareSerial : public Stream
{
  private:
    ring_buffer *_rx_buffer;
    bool transmiting;
    unsigned long serialPort;
  public:
    HardwareSerial(ring_buffer *rx_buffer, uint32_t UARTBase);
    void begin(unsigned long);
    void begin(unsigned long, unsigned long);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool();
};

// Define config for Serial.begin(baud, config);
#define SERIAL_5N1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_NONE)
#define SERIAL_6N1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_NONE)
#define SERIAL_7N1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_NONE)
#define SERIAL_8N1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_NONE)
#define SERIAL_5N2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_NONE)
#define SERIAL_6N2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_NONE)
#define SERIAL_7N2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_NONE)
#define SERIAL_8N2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_NONE)
#define SERIAL_5E1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_EVEN)
#define SERIAL_6E1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_EVEN)
#define SERIAL_7E1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_EVEN)
#define SERIAL_8E1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_EVEN)
#define SERIAL_5E2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_EVEN)
#define SERIAL_6E2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_EVEN)
#define SERIAL_7E2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_EVEN)
#define SERIAL_8E2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_EVEN)
#define SERIAL_5O1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_ODD)
#define SERIAL_6O1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_ODD)
#define SERIAL_7O1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_ODD)
#define SERIAL_8O1  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_1 | xUART_CONFIG_PAR_ODD)
#define SERIAL_5O2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_ODD)
#define SERIAL_6O2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_ODD)
#define SERIAL_7O2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_ODD)
#define SERIAL_8O2  (xUART_CONFIG_WLEN_8 | xUART_CONFIG_STOP_2 | xUART_CONFIG_PAR_ODD)

extern HardwareSerial Serial;

//extern void serialEventRun(void) __attribute__((weak));

#endif
