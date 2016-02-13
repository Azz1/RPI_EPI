#include "Wire.h"

uint8_t WireClass::rxBuffer[BUFFER_LENGTH];
uint8_t WireClass::rxBufferIndex = 0;
uint8_t WireClass::rxBufferLength = 0;

uint8_t WireClass::txAddress = 0;
uint8_t WireClass::txBuffer[BUFFER_LENGTH];
uint8_t WireClass::txBufferIndex = 0;
uint8_t WireClass::txBufferLength = 0;

uint8_t WireClass::transmitting = 0;
void (*WireClass::user_onRequest)(void);
void (*WireClass::user_onReceive)(int);

/*******************************
 *                             *
 * WirePi Class implementation *
 * --------------------------- *
 *******************************/
extern "C" unsigned long WireClass::I2CCallbackFunc(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){
    (void)pvCBData;
	(void)ulEvent;
	(void)pvMsgData;
    uint8_t *s = 0;
    if(ulMsgParam) onRequestService();
    else onReceiveService(s, 2);
    return 0;
}


/******************
 * Public methods *
 ******************/
//Constructor
WireClass::WireClass(){
    i2cPort = sI2C_BASE;
    ulW = 0;
}

/*
 * Initiate the Wire library and join the I2C bus as a master. This should normally be called only once.
 */
void WireClass::begin(void){
    // Enable the i2c&GPIO peripheral
    xSysCtlPeripheralEnable2(i2cPort);
    xSysCtlPeripheralEnable2(xGPIOSPinToPort(sSDA));

    // Congigure the i2c pin
    sPinTypeI2C(i2cPort);
    
    //Initialize I2C Module 100K
    xI2CMasterInit(i2cPort, 100000);
    xI2CMasterEnable(i2cPort);
    
    rxBufferIndex = 0; rxBufferLength = 0;
    txBufferIndex = 0; txBufferLength = 0;
}

/*
 * Initiate the Wire library and join the I2C bus as a slave. This should normally be called only once.
 * Parameters: address: the 7-bit slave address (optional);
 */
void WireClass::begin(uint8_t address){
    // Init the I2C as slave
    //xI2CSlaveInit(i2cPort, address, xI2C_GENERAL_CALL_EN); //mcy
    xI2CSlaveEnable(i2cPort);
    xI2CIntCallbackInit(i2cPort, WireClass::I2CCallbackFunc);
    xI2CSlaveIntEnable(i2cPort, xI2C_SLAVE_INT_DATA | xI2C_MASTER_INT_DATA);
    xIntEnable(xSysCtlPeripheraIntNumGet(i2cPort));
}

void WireClass::begin(int address){
    begin((uint8_t)address);  
}

//Begin a transmission to the I2C slave device with the given address
void WireClass::beginTransmission(uint8_t address){
    // indicate that we are transmitting
    transmitting = 1;
    // set address of targeted slave
    txAddress = address;
    // reset tx buffer iterator vars
    txBufferIndex = 0;
    txBufferLength = 0;
}

void WireClass::beginTransmission(int address){
    beginTransmission((uint8_t)address);
}

//Writes data to the I2C.
size_t WireClass::write(uint8_t data){
    if(transmitting){
        // in master transmitter mode
        // don't bother if buffer is full
        if(txBufferLength >= BUFFER_LENGTH){
            setWriteError();
            return 0;
        }
        // put byte in tx buffer
        txBuffer[txBufferIndex] = data;
        ++txBufferIndex;
        // update amount in buffer   
        txBufferLength = txBufferIndex;
    } else {
        // in slave send mode
        // reply to master
        ulW++;
        if(ulW == 1)
            xI2CMasterWriteS1(i2cPort, (unsigned char)txAddress, data, xfalse);
        else
            xI2CMasterWriteS2(i2cPort, (unsigned char)data, xfalse);
    }
    return 1;  
}

size_t WireClass::send(uint8_t data){
	return write(data);
}
////Writes datas to the I2C.
// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t WireClass::write(const uint8_t *buf, size_t quantity){
    if(transmitting){
        // in master transmitter mode
        for(size_t i = 0; i < quantity; ++i){
            write(buf[i]);
        }
    } else {
        // in slave send mode, reply to master
        ulW++;
        if(ulW == 1)
            xI2CMasterWriteBufS1(i2cPort, (unsigned char)txAddress, (unsigned char*)buf, (unsigned long)quantity, (xtBoolean)xfalse);
        else
            xI2CMasterWriteBufS2(i2cPort, (unsigned char*)buf, (unsigned long)quantity, (xtBoolean)xfalse);
    }
    return quantity;   
}

uint8_t WireClass::endTransmission(uint8_t sendStop)
{
    // transmit buffer (blocking)
    int8_t ret = xI2CMasterWriteBufS1(i2cPort, txAddress, txBuffer, txBufferLength, sendStop);
    // reset tx buffer iterator vars
    txBufferIndex = 0;
    txBufferLength = 0;
    // indicate that we are done transmitting
    transmitting = 0;
    return ret;
}

uint8_t WireClass::endTransmission(){
    return endTransmission(true);
}

uint8_t WireClass::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
    // clamp to buffer length
    if(quantity > BUFFER_LENGTH){
        quantity = BUFFER_LENGTH;
    }
    uint8_t read = xI2CMasterReadBufS1(i2cPort, address, rxBuffer, quantity, sendStop); 
    
    // set rx buffer iterator vars
    rxBufferIndex = 0;
    rxBufferLength = read;

    return read;
}

//Used by the master to request bytes from a slave device
uint8_t WireClass::requestFrom(uint8_t address, uint8_t quantity){
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);  
}

uint8_t WireClass::requestFrom(int address, int quantity){
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t WireClass::requestFrom(int address, int quantity, int sendStop){
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

//Returns the number of bytes available for retrieval with read(). 
int WireClass::available(){
    return rxBufferLength - rxBufferIndex;
}

//Reads a byte that was transmitted from a slave device to a master after a call to WirePi::requestFrom()
int WireClass::read(void){
    int value = -1;
  
    // get each successive byte on each call
    if(rxBufferIndex < rxBufferLength){
        value = rxBuffer[rxBufferIndex];
        ++rxBufferIndex;
    }

    return value;
}

int WireClass::receive(void){
	return read();
}

uint8_t WireClass::reads(char* buf){
    return xI2CMasterReadBufS1(i2cPort, txAddress, (unsigned char*)buf, sizeof(buf), xtrue);
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int WireClass::peek(void)
{
    int value = -1;
  
    if(rxBufferIndex < rxBufferLength){
        value = rxBuffer[rxBufferIndex];
    }

    return value;
}

void WireClass::flush(void)
{
    // XXX: to be implemented.
}

/*******************
 * Private methods *
 *******************/

// behind the scenes function that is called when data is received
void WireClass::onReceiveService(uint8_t* inBytes, int numBytes)
{
    // don't bother if user hasn't registered a callback
    if(!user_onReceive){
        return;
    }
    // don't bother if rx buffer is in use by a master requestFrom() op
    // i know this drops data, but it allows for slight stupidity
    // meaning, they may not have read all the master requestFrom() data yet
    if(rxBufferIndex < rxBufferLength){
        return;
    }
    // copy twi rx buffer into local read buffer
    // this enables new reads to happen in parallel
    for(uint8_t i = 0; i < numBytes; ++i){
        rxBuffer[i] = inBytes[i];    
    }
    // set rx iterator vars
    rxBufferIndex = 0;
    rxBufferLength = numBytes;
    // alert user program
    user_onReceive(numBytes);
}

// behind the scenes function that is called when data is requested
void WireClass::onRequestService(void){
    // don't bother if user hasn't registered a callback
    if(!user_onRequest){
        return;
    }
    // reset tx buffer iterator vars
    // !!! this will kill any pending pre-master sendTo() activity
    txBufferIndex = 0;
    txBufferLength = 0;
    // alert user program
    user_onRequest();
}

/*
 *@brief: onReceive(fun)
 *@Description: Registers a function to be called when a slave device receives a transmission from a master.
 *@param: handler: the function to be called when the slave receives data; this should take a
        single int parameter (the number of bytes read from the master) and return nothing,
        e.g.:  void myHandler(int numBytes)
 *@return: None.
*/
void WireClass::onReceive( void (*function)(int) ){
    user_onReceive = function;
}

/*
 *@brief: Register a function to be called when a master requests data from this slave device.
 *@param handler: the function to be called, takes no parameters and returns nothing, e.g.: void myHandler()
 *@return: None
 */
void WireClass::onRequest( void (*function)(void) ){
    user_onRequest = function;
}


// Preinstantiate Objects //////////////////////////////////////////////////////
WireClass Wire = WireClass();
