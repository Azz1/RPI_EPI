#ifndef _ARDUINO_H_
#define _ARDUINO_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>
#include "bit.h"
#include "Port.h"
#include "Print.h"

typedef uint8_t       boolean;
typedef uint8_t       byte;
typedef uint16_t      word;

#define  F_CPU                          xSysCtlClockGet()
#define  clockCyclesPerMicrosecond()    (F_CPU / 1000000L)
#define  clockCyclesToMicroseconds(a)   ((a) / clockCyclesPerMicrosecond())
#define  microsecondsToClockCycles(a)   ((a) * clockCyclesPerMicrosecond())

#define PI                      3.1415926535897932384626433832795
#define HALF_PI                 1.5707963267948966192313216916398
#define TWO_PI                  6.283185307179586476925286766559
#define DEG_TO_RAD              0.017453292519943295769236907684886
#define RAD_TO_DEG              57.295779513082320876798154814105


//Constraint function
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define max(a,b)                ((a)>(b)?(a):(b))
#define min(a,b)                ((a)<(b)?(a):(b))
#define round(x)                ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg)            ((deg)*DEG_TO_RAD)
#define degrees(rad)            ((rad)*RAD_TO_DEG)
#define sq(x)                   ((x)*(x))

#define lowByte(w)              ((uint8_t) ((w) & 0xff))
#define highByte(w)             ((uint8_t) ((w) >> 8))
#define _BV(bit)                (1 << (bit))
#define bit(b)                  (1UL << (b))
#define bitRead(value, bit)     (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)      ((value) |= (1UL << (bit)))
#define bitClear(value, bit)    ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))


#define isPrintable(x)           (x)
#define isAlpha(x)               (x)
#define isDigit(x)               (x)


typedef enum{
    MSBFIRST = xSPI_MSB_FIRST,  ///< MSB First
    LSBFIRST = xSPI_LSB_FIRST,  ///< LSB First
}cSPIBitOrder;


typedef enum{
    GPIO_FSEL_INPT,   ///< Input
    GPIO_FSEL_OUTP,   ///< Output
} cGPIOFunctionSelect;

typedef enum {
    INPUT,
    OUTPUT,
    INPUT_PULLUP,
    PWM
}Pinmode;

typedef enum {
    LOW  = 0,
    HIGH = 1,
}Digivalue;

typedef enum{
    FALLING = xGPIO_FALLING_EDGE,
    RISING  = xGPIO_RISING_EDGE,
    CHANGE  = xGPIO_BOTH_EDGES,
    //LOW    = xGPIO_LOW_LEVEL,
    //HIGH   = xGPIO_HIGH_LEVEL,
}GPIOIntMode;

typedef enum{
    EXTERNAL,
    DEFAULT,
    INTERNAL,
    INTERNAL1V1,
    INTERNAL2V56,
}AnalogReferType;

#define SS    10
#define SCK   13
#define MOSI  11
#define MISO  12

#define A0  14
#define A1  15
#define A2  16
#define A3  17
#define A4  18
#define A5  19

#define SDA 18
#define SCL 19


class System{
private:
    unsigned long s_ulExtClockMHz;
public:
    System();
    void _init(void);
};


/* Some useful arduino functions */
void shiftOut(uint8_t dataPin, uint8_t clockPin, cSPIBitOrder bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, cSPIBitOrder bitOrder);
void pinMode(int pin, Pinmode mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
unsigned long pulseIn(int pin, int value, unsigned long timeout = 1000);
void analogReference(unsigned char type);
int analogRead (int pin);
void analogWrite (int pin, int value);
void tone(int pin, unsigned long frequency, unsigned long duration = 50);
void noTone(int pin);
unsigned long millis();
unsigned long micros();
void delay(unsigned long millis);
void delayMicroseconds(unsigned long micros);
/*
Re-enables interrupts (after they've been disabled by noInterrupts()).
Interrupts allow certain important tasks to happen in the background
and are enabled by default. Some functions will not work while interrupts
are disabled, and incoming communication may be ignored. Interrupts can
slightly disrupt the timing of code, however, and may be disabled for
particularly critical sections of code.
*/
void interrupts(void); //enable
/*
Disables interrupts (you can re-enable them with interrupts()). Interrupts
allow certain important tasks to happen in the background and are enabled
by default. Some functions will not work while interrupts are disabled, and
incoming communication may be ignored. Interrupts can slightly disrupt the
timing of code, however, and may be disabled for particularly critical sections of code.
*/
void noInterrupts(void); //disable

/*
 *@brief:
 *@param: pin:     the pin number    (Cookie only)
 *@param: function:     the function to call when the interrupt occurs; this function must take no parameters and return nothing. This function is sometimes referred to as an interrupt service routine.
 *@param: mode:    defines when the interrupt should be triggered. Four contstants are predefined as valid values:
          LOW to trigger the interrupt whenever the pin is low,
          CHANGE to trigger the interrupt whenever the pin changes value
          RISING to trigger when the pin goes from low to high,
          FALLING for when the pin goes from high to low.
          HIGH to trigger the interrupt whenever the pin is high.

 *@return: Returns: none

 *@note: Inside the attached function, delay() won't work and the value returned by
       millis() will not increment. Serial data received while in the function may
       be lost. You should declare as volatile any variables that you modify within
       the attached function
*/
void attachInterrupt(int pin, void (*ISR)(void), GPIOIntMode mode);
void detachInterrupt(int pin);

void setup(void);
void loop(void);

long map(long value, long in_min, long in_max, long out_min, long out_max);

//random function
void randomSeed(unsigned int seed);
long random(long howbig);
long random(long min, long max);

unsigned int makeWord(unsigned int w);
unsigned int makeWord(unsigned char h, unsigned char l);
#define word(...) makeWord(__VA_ARGS__)

//CRC
uint16_t inline _crc_xmodem_update(uint16_t crc, uint8_t data);
uint16_t inline _crc_ccitt_update(uint16_t crc, uint8_t data);
uint8_t inline _crc_ibutton_update(uint8_t crc, uint8_t data);
uint16_t inline _crc16_update(uint16_t crc, uint8_t data);
uint16_t crc_xmodem_check (uint8_t *data , uint16_t size);
uint8_t crc_ibutton_checck(uint8_t *data , uint16_t size);
uint16_t crc16_check (uint8_t *data , uint16_t size);
uint16_t crc_ccitt_check(uint8_t *data , uint16_t size);

extern System SYS;

#endif
