#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <stdio.h>
#include <Arduino.h>

/// \brief cSPIMode
/// Specify the SPI data mode
typedef enum{
    SPI_MODE0 = xSPI_MOTO_FORMAT_MODE_0,  ///< CPOL = 0, CPHA = 0
    SPI_MODE1 = xSPI_MOTO_FORMAT_MODE_1,  ///< CPOL = 0, CPHA = 1
    SPI_MODE2 = xSPI_MOTO_FORMAT_MODE_2,  ///< CPOL = 1, CPHA = 0
    SPI_MODE3 = xSPI_MOTO_FORMAT_MODE_3,  ///< CPOL = 1, CPHA = 1
}cSPIMode;

typedef enum{
    cSPI_SS_HARDWARE = xSPI_SS_HARDWARE,
    cSPI_SS_SOFTWARE = xSPI_SS_SOFTWARE,
}cSPISSMode;

typedef enum{
    SPI_CLOCK_DIV2   = 2,   //8MHz
    SPI_CLOCK_DIV4   = 4,   //4MHz
    SPI_CLOCK_DIV8   = 8,   //2MHz
    SPI_CLOCK_DIV16  = 16,  //1MHz
    SPI_CLOCK_DIV32  = 32,  //500KHz
    SPI_CLOCK_DIV64  = 64,  //250KHz
    SPI_CLOCK_DIV128 = 128, //125KHz
}cSPIDivClock;

class SPIClass{
  private:
        unsigned long spiPort;
        unsigned long ulDataWidth;
        unsigned long ulDataFormat;
        unsigned long ulDataMode;
        unsigned long spiClock;
  public:
        SPIClass();
        void begin(int spiClock = 1000000);
        void end();
        void setBitOrder(cSPIBitOrder order);
        void setClockDivider(uint16_t divider);
        void setDataMode(cSPIMode ulMode);
        void chipSelect(cSPISSMode cs);
        uint8_t transfer(uint8_t value);
        void transfernb(char* tbuf, char* rbuf, uint32_t len);
        inline static void attachInterrupt();
        inline static void detachInterrupt(); // Default
};

extern SPIClass SPI;

void SPIClass::attachInterrupt() {
  //SPCR |= _BV(SPIE);
}

void SPIClass::detachInterrupt() {
  //SPCR &= ~_BV(SPIE);
}

#endif
