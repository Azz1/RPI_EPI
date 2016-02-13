#ifndef _SOFTWARESERIAL_H_
#define _SOFTWARESERIAL_H_

#include <inttypes.h>
#include "Stream.h"

/*
 * IMPORTANT:
 *
 * This class documented "by hand" (i.e., not using Doxygen) in the
 * leaflabs-docs/ repository.
 *
 * If you alter the public HardwareSerial interface, you MUST update
 * the documentation accordingly.
 */

#define _SS_MAX_RX_BUFF 64 // RX buffer size

class SoftwareSerial : public Stream
{
    private:
        // per object data
        uint8_t _receivePin;
        uint8_t _transmitPin;

        uint16_t _rx_delay_centering;
        uint16_t _rx_delay_intrabit;
        uint16_t _rx_delay_stopbit;
        uint16_t _tx_delay;

        uint16_t _buffer_overflow:1;
        uint16_t _inverse_logic:1;

        // static data
        static char _receive_buffer[_SS_MAX_RX_BUFF]; 
        static volatile uint8_t _receive_buffer_tail;
        static volatile uint8_t _receive_buffer_head;
        static SoftwareSerial *active_object;

        // private methods
        void recv();
        uint8_t rx_pin_read();
        void tx_pin_write(uint8_t pin_state);
        void setTX(uint8_t transmitPin);
        void setRX(uint8_t receivePin);

        // private static method for timing
        static inline void tunedDelay(uint16_t delay);

    public:
        // public methods
        SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
        ~SoftwareSerial();
        void begin(uint32_t speed);
        bool listen();
        void end();
        bool isListening() { return this == active_object; }
        bool overflow() { bool ret = _buffer_overflow; _buffer_overflow = false; return ret; }
        int peek();

        virtual size_t write(uint8_t byte);
        virtual int read();
        virtual int available();
        virtual void flush();
  
        using Print::write;

        // public only for easy access by interrupt handlers
        static inline void handle_interrupt();
};

#endif
