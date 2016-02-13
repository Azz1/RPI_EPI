#ifndef _WIRE_H_
#define _WIRE_H_

#include <stdio.h>
#include "Arduino.h"
#include "Stream.h"

#define BUFFER_LENGTH 32

/* WirePi Class
 * Class that provides the functionality of arduino Wire library
 */
class WireClass : public Stream {
    private:
        unsigned long i2cPort;
        unsigned int ulW;
        static uint8_t rxBuffer[];
        static uint8_t rxBufferIndex;
        static uint8_t rxBufferLength;
  
        static uint8_t txAddress;
        static uint8_t txBuffer[];
        static uint8_t txBufferIndex;
        static uint8_t txBufferLength;

        static uint8_t transmitting;
        static void (*user_onRequest)(void);  // sets function called on slave read
        static void (*user_onReceive)(int);   // sets function called on slave write
        static void onRequestService(void);
        static void onReceiveService(uint8_t*, int);
        static unsigned long I2CCallbackFunc(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData);
    public:
        WireClass();
        void begin();
        void begin(uint8_t);
        void begin(int);  //带地址参数就是从机，不带就是主机
        void beginTransmission(uint8_t);
        void beginTransmission(int);
        uint8_t write(char * buf, uint32_t len);
        uint8_t endTransmission(void);
        uint8_t endTransmission(uint8_t);
        uint8_t requestFrom(uint8_t, uint8_t);
        uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
        uint8_t requestFrom(int, int);
        uint8_t requestFrom(int, int, int);
        virtual size_t write(uint8_t);
        virtual size_t write(const uint8_t*, size_t);
        virtual int available(void);
        virtual int read(void);
        virtual int peek(void);
        virtual void flush(void);
        uint8_t reads(char* buf);
        void onReceive(void (* cPiI2CRecvHandler)(int)); //从机接收主机发来的数据
        void onRequest(void (* cPiI2CReqHandle)(void)); //主机请求从机发送数据
        
        size_t send(uint8_t);
        int receive(void);

        inline size_t write(unsigned long n) { return write((uint8_t)n); }
        inline size_t write(long n) { return write((uint8_t)n); }
        inline size_t write(unsigned int n) { return write((uint8_t)n); }
        inline size_t write(int n) { return write((uint8_t)n); }
        using Print::write;
};

extern WireClass Wire;

#endif
