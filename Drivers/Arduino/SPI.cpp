#include "SPI.h"

/*******************************
 *                             *
 * SPIPi Class implementation  *
 * --------------------------- *
 *******************************/

/******************
 * Public methods *
 ******************/
SPIClass::SPIClass(){
    spiPort = sSPI_BASE;
}

//Initializes the SPI bus by setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high.
void SPIClass::begin(int spiClock){
    // Enable Peripheral SPIx
    xSysCtlPeripheralEnable2(spiPort);
    xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD13));

    // Configure Some GPIO pins as SPIx Mode
    sPinTypeSPI(spiPort);
    ulDataWidth = xSPI_DATA_WIDTH8;
    ulDataFormat = SPI_MODE0;
    ulDataMode = MSBFIRST;
    xSPIConfigSet(spiPort, spiClock, xSPI_MOTO_FORMAT_MODE_0 | xSPI_DATA_WIDTH8 | xSPI_MSB_FIRST | xSPI_MODE_MASTER);
    xSPISSSet(spiPort, xSPI_SS_HARDWARE, xSPI_SS0);
    xSPIEnable(spiPort);
}

//Disables the SPIx bus
void SPIClass::end(){
    xSPIDisable(spiPort);
}

//Sets the order of the bits shifted out of and into the SPIx bus, either LSBFIRST or MSBFIRST.
void SPIClass::setBitOrder(cSPIBitOrder order){
    xSPIDisable(spiPort);
    ulDataMode = order;
    xSPIConfigSet(spiPort, spiClock, ulDataFormat | ulDataWidth | ulDataMode | xSPI_MODE_MASTER);
    xSPIEnable(spiPort);
}

/* Sets the SPIx clock divider relative to the system clock.
 * the dividers available are 2, 4, 8, 16, 32, 64 or 128. The default setting is SPI_CLOCK_DIV4,
 * which sets the clock to 4 MHz like other Arduino boards.
 */
void SPIClass::setClockDivider(uint16_t divider){
    xSPIDisable(spiPort);
    spiClock = 16000000 / divider;
    xSPIConfigSet(spiPort, spiClock, ulDataFormat | ulDataWidth | ulDataMode | xSPI_MODE_MASTER);
    xSPIEnable(spiPort);
}

//Sets the SPIx data mode: SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3
void SPIClass::setDataMode(cSPIMode ulMode){
    xSPIDisable(spiPort);
    ulDataFormat = ulMode;
    xSPIConfigSet(spiPort, spiClock, ulDataFormat | ulDataWidth | ulDataMode | xSPI_MODE_MASTER);
    xSPIEnable(spiPort);
}

// Writes (and reads) a single byte to SPIx
uint8_t SPIClass::transfer(uint8_t value){
    unsigned long ulReadTemp = xSPISingleDataReadWrite(spiPort, value);
    return ulReadTemp;
}

// Writes (and reads) a number of bytes to SPIx
void SPIClass::transfernb(char* tbuf, char* rbuf, uint32_t len){
    unsigned int t_size = strlen(tbuf);
    while(t_size--){
        transfer(*(tbuf++));
    }
    while(len--){
        *(rbuf++) = transfer(0xff);
    }
}

void SPIClass::chipSelect(cSPISSMode cs){
    xSPISSSet(spiPort, cs, xSPI_SS0);
}

SPIClass SPI = SPIClass();
