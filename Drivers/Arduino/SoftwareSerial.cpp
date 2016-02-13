#include <Arduino.h>
#include <SoftwareSerial.h>

//#define _DEBUG 0
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13

//
// Lookup table
//
typedef struct _DELAY_TABLE{
    uint32_t baud;
    uint16_t rx_delay_centering;
    uint16_t rx_delay_intrabit;
    uint16_t rx_delay_stopbit;
    uint16_t tx_delay;
} DELAY_TABLE;

static const DELAY_TABLE table[] = {
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   1,           7,         8,     7,      }, //8.
  { 57600,    2,          16,        18,     17,     }, //17.375
  { 38400,    3,          25,        28,     26,     }, //26
  { 31250,    4,          31,        35,     32,     }, //32
  { 28800,    4,          98,        98,     35,     }, //34.6875 
  { 19200,    4,          55,        60,     55,     }, //52.06
  { 14400,    7,          75,        80,     71,     }, //69.375
  { 9600,     10,         110,       120,    108,    }, //104.1875
  { 4800,     15,         225,       230,    222,    }, //208.3125
  { 2400,     20,         455,       460,    450,    }, //416.6875
  { 1200,     30,         915,       920,    910,    }, //833.5
  { 600,      40,        1840,      1900,   1830,   }, 
  { 300,      50,        3750,      3800,   3700,   },
};

//
// Statics
//
SoftwareSerial *SoftwareSerial::active_object = 0;
char SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerial::_receive_buffer_head = 0;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
inline void DebugPulse(uint8_t pin, uint8_t count)
{
#if _DEBUG
    uint32_t val = digitalRead(pin);
    while(count--){
        digitalWrite(pin, val|1);
        digitalWrite(pin, val);
    }
#endif
}

//
// Private methods
//

/* static */ 
inline void SoftwareSerial::tunedDelay(uint16_t us) {
    us *= 11;
    xSysCtlDelay(us);
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
bool SoftwareSerial::listen()
{
    if (active_object != this){
        _buffer_overflow = false;
        _receive_buffer_head = _receive_buffer_tail = 0;
        active_object = this;
        return true;
    }

    return false;
}

//
// The receive routine called by the interrupt handler
//
void SoftwareSerial::recv()
{
    uint8_t d = 0;

    // If RX line is high, then we don't see any start bit
    // so interrupt is probably not for us
    if (_inverse_logic ? rx_pin_read() : !rx_pin_read()){
        // Wait approximately 1/2 of a bit width to "center" the sample
        tunedDelay(_rx_delay_centering);
        DebugPulse(_DEBUG_PIN2, 1);

        // Read each of the 8 bits
        for (uint8_t i=0x1; i; i <<= 1){
            tunedDelay(_rx_delay_intrabit);
            DebugPulse(_DEBUG_PIN2, 1);
            uint8_t noti = ~i;
            if (rx_pin_read())
                d |= i;
            else // else clause added to ensure function timing is ~balanced
                d &= noti;
        }

        // skip the stop bit
        tunedDelay(_rx_delay_stopbit);
        DebugPulse(_DEBUG_PIN2, 1);

        if (_inverse_logic)
            d = ~d;

        // if buffer full, set the overflow flag and return
        if ((_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF != _receive_buffer_head){
            // save new data in buffer: tail points to where byte goes
            _receive_buffer[_receive_buffer_tail] = d; // save new byte
            _receive_buffer_tail = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
        } else {
#if _DEBUG // for scope: pulse pin as overflow indictator
            DebugPulse(_DEBUG_PIN1, 1);
#endif
            _buffer_overflow = true;
        }
    }
}

void SoftwareSerial::tx_pin_write(uint8_t pin_state)
{
	digitalWrite(_transmitPin, pin_state);
}

uint8_t SoftwareSerial::rx_pin_read()
{
	return digitalRead(_receivePin);
}

//
// Interrupt handling
//

/* static */
inline void SoftwareSerial::handle_interrupt()
{
    if (active_object){
        active_object->recv();
    }
}

void PinCallback(void){
    SoftwareSerial::handle_interrupt();
}

//
// Constructor
//
SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */) : 
    _rx_delay_centering(0),
    _rx_delay_intrabit(0),
    _rx_delay_stopbit(0),
    _tx_delay(0),
    _buffer_overflow(false),
    _inverse_logic(inverse_logic)
    {
        this->_receivePin = receivePin;
        this->_transmitPin = transmitPin;
        setTX(transmitPin);
        setRX(receivePin);
    }

//
// Destructor
//
SoftwareSerial::~SoftwareSerial()
{
    end();
}

void SoftwareSerial::setTX(uint8_t tx)
{
    pinMode(tx, OUTPUT);
    digitalWrite(tx, HIGH);
}

void SoftwareSerial::setRX(uint8_t rx)
{
    pinMode(rx, INPUT);
    if (!_inverse_logic)
        digitalWrite(rx, HIGH);  // pullup for normal logic!
}

//
// Public methods
//

void SoftwareSerial::begin(uint32_t speed)
{
    _rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;

    for (unsigned i=0; i<sizeof(table)/sizeof(table[0]); ++i){
    	uint32_t baud = table[i].baud;
        if (baud == speed){
            _rx_delay_centering = table[i].rx_delay_centering;
            _rx_delay_intrabit = table[i].rx_delay_intrabit;
            _rx_delay_stopbit = table[i].rx_delay_stopbit;
            _tx_delay = table[i].tx_delay;
            break;
        }
    }

    // Set up RX interrupts, but only if we have a valid RX baud rate
    if (_rx_delay_stopbit){
        attachInterrupt(_receivePin, PinCallback, CHANGE);
        tunedDelay(_tx_delay); // if we were low this establishes the end
    }

#if _DEBUG
    pinMode(_DEBUG_PIN1, OUTPUT);
    pinMode(_DEBUG_PIN2, OUTPUT);
#endif

    listen();
}

void SoftwareSerial::end()
{
    detachInterrupt(_receivePin);
}


// Read data from buffer
int SoftwareSerial::read()
{
    if (!isListening())
        return -1;

    // Empty buffer?
    if (_receive_buffer_head == _receive_buffer_tail)
        return -1;

    // Read from "head"
    uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
    _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
    return d;
}

int SoftwareSerial::available()
{
    if (!isListening())
        return 0;

    return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b)
{
    if (_tx_delay == 0) {
        setWriteError();
        return 0;
    }

    //noInterrupts(); // turn off interrupts for a clean txmit

    // Write the start bit
    tx_pin_write(_inverse_logic ? HIGH : LOW);
    tunedDelay(_tx_delay);

    // Write each of the 8 bits
    if (_inverse_logic){
        for (byte mask = 0x01; mask; mask <<= 1){
            if (b & mask) // choose bit
                tx_pin_write(LOW); // send 1
            else
                tx_pin_write(HIGH); // send 0
    
            tunedDelay(_tx_delay);
        }

        tx_pin_write(LOW); // restore pin to natural state
    } else {
        for (byte mask = 0x01; mask; mask <<= 1){
            if (b & mask) // choose bit
                tx_pin_write(HIGH); // send 1
            else
                tx_pin_write(LOW); // send 0
    
            tunedDelay(_tx_delay);
        }

        tx_pin_write(HIGH); // restore pin to natural state
    }

    //interrupts(); // turn interrupts back on
    tunedDelay(_tx_delay);
  
    return 1;
}

void SoftwareSerial::flush()
{
    if (!isListening())
        return;
  
    noInterrupts();
    _receive_buffer_head = _receive_buffer_tail = 0;
    interrupts();
}

int SoftwareSerial::peek()
{
    if (!isListening())
        return -1;

    // Empty buffer?
    if (_receive_buffer_head == _receive_buffer_tail)
        return -1;

    // Read from "head"
    return _receive_buffer[_receive_buffer_head];
}
