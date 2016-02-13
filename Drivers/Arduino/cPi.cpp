#include "Arduino.h"

/*************************************
 *                                   *
 * SYSTEM init Class implementation  *
 * --------------------------------- *
 *************************************/

/******************
 * Public methods *
 ******************/
System::System(){
    // Initionalize system clock.
    xSysCtlClockSet(72000000,  xSYSCTL_OSC_MAIN | xSYSCTL_XTAL_8MHZ);
    //xSysCtlClockSet(8000000,  xSYSCTL_OSC_INT | xSYSCTL_INT_8MHZ);
    xSysCtlDelay(10000);

    xSysCtlPeripheralEnable(SYSCTL_PERIPH_AFIO);
    xHWREG(AFIO_MAPR) = 0x02000000; // Disable the JTAG and release the PB3 and PB5. 

    //Enable systick, Interrupt period = 1ms
    xSysTickIntEnable();
    xSysTickPeriodSet(72000);
    xSysTickEnable();
}

void System::_init(){

}

/********** FUNCTIONS OUTSIDE CLASSES **********/
static volatile unsigned long ulCnt = 0;
extern "C" void SysTickIntHandler(){
    ulCnt++;
}

/*
 * @brief:  Returns the number of milliseconds since the Arduino board began running the current program.
 *          This number will overflow (go back to zero), after approximately 50 days.
 * @param:  None.
 * @return: Number of milliseconds since the program started (unsigned long).
 *
 */
unsigned long millis(){
    return ulCnt;
}

unsigned long micros(){
    return 0;
}

// Sleep the specified milliseconds
void delay(unsigned long millis){
    if(millis >= (0xffffffff - ulCnt)) ulCnt = 0;
    unsigned long tmp = ulCnt + millis;
    while(tmp != ulCnt);
}

void delayMicroseconds(unsigned long micros){
    xSysCtlDelay(micros * 18);
}

/* Shifts out a byte of data one bit at a time.Starts from either the most (i.e. the leftmost) or
 * least (rightmost) significant bit. Each bit is written in turn to a data pin, after which a
 * clock pin is pulsed (taken high, then low) to indicate that the bit is available.
 */
void shiftOut(uint8_t dataPin, uint8_t clockPin, cSPIBitOrder bitOrder, uint8_t val){
    uint8_t i;

    for (i = 0; i < 8; i++)  {
        if (bitOrder == LSBFIRST)
            digitalWrite(dataPin, !!(val & (1 << i)));
        else
            digitalWrite(dataPin, !!(val & (1 << (7 - i))));

        digitalWrite(clockPin, HIGH);
        digitalWrite(clockPin, LOW);
    }
}

/* Shifts in a byte of data one bit at a time. Starts from either the most (i.e. the leftmost) or
 * least (rightmost) significant bit. For each bit, the clock pin is pulled high, the next bit is
 * read from the data line, and then the clock pin is taken low.
 */
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, cSPIBitOrder bitOrder) {
    uint8_t i, value = 0;

    for (i = 0; i < 8; ++i) {
        digitalWrite(clockPin, HIGH);
        if (bitOrder == LSBFIRST)
            value |= digitalRead(dataPin) << i;
        else
            value |= digitalRead(dataPin) << (7 - i);
        digitalWrite(clockPin, LOW);
    }
    return value;
}

// Configures the specified pin to behave either as an input or an output
void pinMode(int pin, Pinmode mode){
    if(mode == OUTPUT){
        switch(pin){
            case 2:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD2));  xGPIOSPinTypeGPIOOutput(sD2);  break;
            case 3:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD3));  xGPIOSPinTypeGPIOOutput(sD3);  break;
            case 4:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD4));  xGPIOSPinTypeGPIOOutput(sD4);  break;
            case 5:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD5));  xGPIOSPinTypeGPIOOutput(sD5);  break;
            case 6:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD6));  xGPIOSPinTypeGPIOOutput(sD6);  break;
            case 7:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD7));  xGPIOSPinTypeGPIOOutput(sD7);  break;
            case 8:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD8));  xGPIOSPinTypeGPIOOutput(sD8);  break;
            case 9:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD9));  xGPIOSPinTypeGPIOOutput(sD9);  break;
            case 10: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD10)); xGPIOSPinTypeGPIOOutput(sD10); break;
            case 11: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD11)); xGPIOSPinTypeGPIOOutput(sD11); break;
            case 12: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD12)); xGPIOSPinTypeGPIOOutput(sD12); break;
            case 13: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD13)); xGPIOSPinTypeGPIOOutput(sD13); break;
            case 14: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD14)); xGPIOSPinTypeGPIOOutput(sD14); break;
            case 15: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD15)); xGPIOSPinTypeGPIOOutput(sD15); break;
        }
    }
    if(mode == INPUT){
        switch(pin){
            case 2:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD2));  xGPIOSPinTypeGPIOInput(sD2);  break;
            case 3:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD3));  xGPIOSPinTypeGPIOInput(sD3);  break;
            case 4:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD4));  xGPIOSPinTypeGPIOInput(sD4);  break;
            case 5:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD5));  xGPIOSPinTypeGPIOInput(sD5);  break;
            case 6:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD6));  xGPIOSPinTypeGPIOInput(sD6);  break;
            case 7:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD7));  xGPIOSPinTypeGPIOInput(sD7);  break;
            case 8:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD8));  xGPIOSPinTypeGPIOInput(sD8);  break;
            case 9:  xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD9));  xGPIOSPinTypeGPIOInput(sD9);  break;
            case 10: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD10)); xGPIOSPinTypeGPIOInput(sD10); break;
            case 11: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD11)); xGPIOSPinTypeGPIOInput(sD11); break;
            case 12: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD12)); xGPIOSPinTypeGPIOInput(sD12); break;
            case 13: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD13)); xGPIOSPinTypeGPIOInput(sD13); break;
            case 14: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD14)); xGPIOSPinTypeGPIOInput(sD14); break;
            case 15: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD15)); xGPIOSPinTypeGPIOInput(sD15); break;
        }
    }
}

// Write a HIGH or a LOW value to a digital pin
void digitalWrite(int pin, int value){
    if(value == HIGH){
        switch(pin){
            case 2:  xGPIOSPinWrite(sD2, 1);  break;
            case 3:  xGPIOSPinWrite(sD3, 1);  break;
            case 4:  xGPIOSPinWrite(sD4, 1);  break;
            case 5:  xGPIOSPinWrite(sD5, 1);  break;
            case 6:  xGPIOSPinWrite(sD6, 1);  break;
            case 7:  xGPIOSPinWrite(sD7, 1);  break;
            case 8:  xGPIOSPinWrite(sD8, 1);  break;
            case 9:  xGPIOSPinWrite(sD9, 1);  break;
            case 10: xGPIOSPinWrite(sD10, 1); break;
            case 11: xGPIOSPinWrite(sD11, 1); break;
            case 12: xGPIOSPinWrite(sD12, 1); break;
            case 13: xGPIOSPinWrite(sD13, 1); break;
            case 14: xGPIOSPinWrite(sD14, 1); break;
            case 15: xGPIOSPinWrite(sD15, 1); break;
        }
    } else if(value == LOW){
        switch(pin){
            case 2:  xGPIOSPinWrite(sD2, 0);  break;
            case 3:  xGPIOSPinWrite(sD3, 0);  break;
            case 4:  xGPIOSPinWrite(sD4, 0);  break;
            case 5:  xGPIOSPinWrite(sD5, 0);  break;
            case 6:  xGPIOSPinWrite(sD6, 0);  break;
            case 7:  xGPIOSPinWrite(sD7, 0);  break;
            case 8:  xGPIOSPinWrite(sD8, 0);  break;
            case 9:  xGPIOSPinWrite(sD9, 0);  break;
            case 10: xGPIOSPinWrite(sD10, 0); break;
            case 11: xGPIOSPinWrite(sD11, 0); break;
            case 12: xGPIOSPinWrite(sD12, 0); break;
            case 13: xGPIOSPinWrite(sD13, 0); break;
            case 14: xGPIOSPinWrite(sD14, 0); break;
            case 15: xGPIOSPinWrite(sD15, 0); break;
        }
    } else if(value == PWM){
	  switch(pin){
		  case 3:  sD3PinTypePWM(); break;
		  case 5:  sD5PinTypePWM(); break;
		  case 6:  sD6PinTypePWM(); break;
		  case 9:  sD9PinTypePWM(); break;
		  case 10: sD10PinTypePWM(); break;
		  case 11: sD11PinTypePWM(); break;
	  }
	}
}

// Reads the value from a specified digital pin, either HIGH or LOW.
int digitalRead(int pin){
    unsigned char value=0;
    switch(pin){
        case 2:  if(xGPIOSPinRead(sD2))  value = HIGH; else value = LOW; break;
        case 3:  if(xGPIOSPinRead(sD3))  value = HIGH; else value = LOW; break;
        case 4:  if(xGPIOSPinRead(sD4))  value = HIGH; else value = LOW; break;
        case 5:  if(xGPIOSPinRead(sD5))  value = HIGH; else value = LOW; break;
        case 6:  if(xGPIOSPinRead(sD6))  value = HIGH; else value = LOW; break;
        case 7:  if(xGPIOSPinRead(sD7))  value = HIGH; else value = LOW; break;
        case 8:  if(xGPIOSPinRead(sD8))  value = HIGH; else value = LOW; break;
        case 9:  if(xGPIOSPinRead(sD9))  value = HIGH; else value = LOW; break;
        case 10: if(xGPIOSPinRead(sD10)) value = HIGH; else value = LOW; break;
        case 11: if(xGPIOSPinRead(sD11)) value = HIGH; else value = LOW; break;
        case 12: if(xGPIOSPinRead(sD12)) value = HIGH; else value = LOW; break;
        case 13: if(xGPIOSPinRead(sD13)) value = HIGH; else value = LOW; break;
        case 14: if(xGPIOSPinRead(sD14)) value = HIGH; else value = LOW; break;
        case 15: if(xGPIOSPinRead(sD15)) value = HIGH; else value = LOW; break;
    }
    return value;
}


unsigned long pulseIn(int pin, int value, unsigned long timeout){
    unsigned long ulStartTime = 0, ulStopTime = 0;
    switch(pin){
        case 2:
            if(xGPIOSPinRead(sD2) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD2) != value) && ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD2) != value) return 0;
            }break;
        case 3:
            if(xGPIOSPinRead(sD3) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD3) != value) || ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD3) != value) return 0;
            }break;
        case 4:
            if(xGPIOSPinRead(sD4) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD4) != value) || ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD4) != value) return 0;
            }break;
        case 5:
            if(xGPIOSPinRead(sD5) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD5) != value) || ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD5) != value) return 0;
            }break;
        case 6:
            if(xGPIOSPinRead(sD6) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD6) != value) || ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD6) != value) return 0;
            }break;
        case 7:
            if(xGPIOSPinRead(sD7) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD7) != value) || ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD7) != value) return 0;
            }break;
        case 8:
            if(xGPIOSPinRead(sD8) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD8) != value) || ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD8) != value) return 0;
            }break;
        case 9:
            if(xGPIOSPinRead(sD9) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD9) != value) || ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD9) != value) return 0;
            }break;
        case 10:
            if(xGPIOSPinRead(sD10) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD10) != value) || ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD10) != value) return 0;
            }break;
        case 11:
            if(xGPIOSPinRead(sD11) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD11) != value) || ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD11) != value) return 0;
            }break;
        case 12:
            if(xGPIOSPinRead(sD12) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD12) != value) || ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD12) != value) return 0;
            }break;
        case 13:
            if(xGPIOSPinRead(sD13) != value){
                ulStartTime = millis();
                while((xGPIOSPinRead(sD13) != value) || ((millis() - ulStartTime) <= timeout));
                ulStopTime = millis();
                if(xGPIOSPinRead(sD13) != value) return 0;
            }break;
    }
    return (ulStopTime - ulStartTime);
}

/*
 * @brief: Configures the reference voltage used for analog input (i.e. the value used as the top of the input range).
 * @param: type DEFAULT: the default analog reference of 5 volts (on 5V Arduino boards) or 3.3 volts (on 3.3V Arduino boards)
 *              INTERNAL: an built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328 and 2.56 volts on the ATmega8 (not available on the Arduino Mega)
 *              INTERNAL1V1: a built-in 1.1V reference (Arduino Mega only)
 *              INTERNAL2V56: a built-in 2.56V reference (Arduino Mega only)
 *              EXTERNAL: the voltage applied to the AREF pin (0 to 5V only) is used as the reference.
 *
 *@return: None.
 */
void analogReference(unsigned char type){

}

int analogRead (int pin){
    unsigned long ulData[8];

    // Enable Peripheral ADC
    xSysCtlPeripheralEnable2(sADC_BASE);

    // Set ADCCLK prescaler, ADCCLK=PCLK2(max 72MHz)/DIV(DIV:2,4,6,8)
    // You should set ADCCLK < 14MHz to ensure the accuracy of ADC
    xSysCtlPeripheralClockSourceSet(xSYSCTL_ADC0_MAIN, 8);

    // Configure Some GPIO pins as SPI Mode
    switch(pin){
        case A0: xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA0)); sA0PinTypeADC(); break;
        case A1: xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA1)); sA1PinTypeADC(); break;
        case A2: xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA2)); sA2PinTypeADC(); break;
        case A3: xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA3)); sA3PinTypeADC(); break;
        case A4: xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA4)); sA4PinTypeADC(); break;
        case A5: xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA5)); sA5PinTypeADC(); break;
    }

    // The two sentences below configure ADC to scan mode, continuous convert, software trigger.
    xADCConfigure(xADC1_BASE, xADC_MODE_SCAN_SINGLE_CYCLE, ADC_TRIGGER_PROCESSOR);
    xADCConfigure(sADC_BASE, xADC_MODE_SCAN_CONTINUOUS, ADC_TRIGGER_PROCESSOR);

    // Enable the channel
    switch(pin){
        case A0: xADCStepConfigure(sADC_BASE, 0, ADC_CHANNEL_0); break;
        case A1: xADCStepConfigure(sADC_BASE, 1, ADC_CHANNEL_1); break;
        case A2: xADCStepConfigure(sADC_BASE, 2, ADC_CHANNEL_4); break;
        case A3: xADCStepConfigure(sADC_BASE, 3, ADC_CHANNEL_8); break;
        case A4: xADCStepConfigure(sADC_BASE, 4, ADC_CHANNEL_11); break;
        case A5: xADCStepConfigure(sADC_BASE, 5, ADC_CHANNEL_10); break;
    }

    // Enable the adc
    xADCEnable(sADC_BASE);
    
    // Read the convert value
    xADCDataGet(sADC_BASE, ulData);
    return ulData[pin - A0] * 3300 / 4095;
}

/*
 *@brief: analogWrite(pin, value)
 *@param: pin the pin to write to.
 *@param: value the duty cycle: between 0 (always off) and 255 (always on).       
 *@note: The frequency of the PWM signal is approximately 490 Hz.
 *@return: nothing
*/
void analogWrite(int pin, int value){
    unsigned long ulPWMBase = PinToPWMBase(pin), ulChannel = 0;

    xSysCtlPeripheralEnable2(ulPWMBase);
    
    // Set GPIO Pin as PWM and enable PWM
    switch(pin){
        case 3: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD3)); sD3PinTypePWM(); ulChannel = 1; break;
        case 5: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD5)); sD5PinTypePWM(); ulChannel = 3; break;
        case 6: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD6)); sD6PinTypePWM(); ulChannel = 4; break;
        case 9: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD9)); sD9PinTypePWM(); ulChannel = 5; break;
    }

    // Set invert, dead zone and mode
    xPWMInitConfigure(ulPWMBase, ulChannel, xPWM_TOGGLE_MODE | xPWM_OUTPUT_INVERTER_DIS | xPWM_DEAD_ZONE_DIS);

    // Set CNR, Prescale and Divider  
    xPWMFrequencyConfig(ulPWMBase, ulChannel, 0x000B03E7);

    // Set CMR
    xPWMDutySet(ulPWMBase, ulChannel, value*100/255);

    // Set output enable
    xPWMOutputEnable(ulPWMBase, ulChannel);

    // start pwm
    xPWMStart(ulPWMBase, ulChannel);
}

/*
 *@brief: Generates a square wave of the specified frequency (and 50% duty cycle) on a pin. 
          A duration can be specified, otherwise the wave continues until a call to noTone(). 
 *@param: pin the pin on which to generate the tone
 *@param: frequency: the frequency of the tone in hertz - unsigned int
 *@param: duration: the duration of the tone in milliseconds (optional) - unsigned long
 *@return: nothing
 */
void tone(int pin, unsigned long frequency, unsigned long duration) {
    unsigned long ulPWMBase = PinToPWMBase(pin), ulChannel = 0;

    xSysCtlPeripheralEnable2(ulPWMBase);
    
    // Set GPIO Pin as PWM and enable PWM
    switch(pin){
        case 3: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD3)); sD3PinTypePWM(); ulChannel = 1; break;
        case 5: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD5)); sD5PinTypePWM(); ulChannel = 3; break;
        case 6: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD6)); sD6PinTypePWM(); ulChannel = 4; break;
        case 9: xSysCtlPeripheralEnable2(xGPIOSPinToPort(sD9)); sD9PinTypePWM(); ulChannel = 5; break;
    }

    // Set invert, dead zone and mode
    xPWMInitConfigure(ulPWMBase, ulChannel, xPWM_TOGGLE_MODE | xPWM_OUTPUT_INVERTER_DIS | xPWM_DEAD_ZONE_DIS);

    // Set CNR, Prescale and Divider
    xPWMFrequencySet(ulPWMBase, ulChannel, frequency);
    
    // Set CMR
    xPWMDutySet(ulPWMBase, ulChannel, duration);

    // Set output enable
    xPWMOutputEnable(ulPWMBase, ulChannel);

    // start pwm
    xPWMStart(ulPWMBase, ulChannel);  
}

void noTone(int pin){
    xSysCtlPeripheralDisable2(PinToPWMBase(pin));
    switch(pin){
        case 3: xSysCtlPeripheralDisable2(xGPIOSPinToPort(sD3)); break;
        case 5: xSysCtlPeripheralDisable2(xGPIOSPinToPort(sD5)); break;
        case 6: xSysCtlPeripheralDisable2(xGPIOSPinToPort(sD6)); break;
        case 9: xSysCtlPeripheralDisable2(xGPIOSPinToPort(sD9)); break;
    }
}

void interrupts(void){
    xIntMasterEnable();
}

void noInterrupts(void){
    xIntMasterDisable();
}


static void (*arduPinIntFun[16])(void);
extern "C" {static unsigned long arduPin2_Callback (void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[2]();  return 0;}}
extern "C" {static unsigned long arduPin3_Callback (void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[3]();  return 0;}}
extern "C" {static unsigned long arduPin4_Callback (void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[4]();  return 0;}}
extern "C" {static unsigned long arduPin5_Callback (void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[5]();  return 0;}}
extern "C" {static unsigned long arduPin6_Callback (void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[6]();  return 0;}}
extern "C" {static unsigned long arduPin7_Callback (void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[7]();  return 0;}}
extern "C" {static unsigned long arduPin8_Callback (void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[8]();  return 0;}}
extern "C" {static unsigned long arduPin9_Callback (void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[9]();  return 0;}}
extern "C" {static unsigned long arduPin10_Callback(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[10](); return 0;}}
extern "C" {static unsigned long arduPin11_Callback(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[11](); return 0;}}
extern "C" {static unsigned long arduPin12_Callback(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[12](); return 0;}}
extern "C" {static unsigned long arduPin13_Callback(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData){(void)pvCBData; (void)ulEvent, (void)ulMsgParam, (void)pvMsgData; arduPinIntFun[13](); return 0;}}

void attachInterrupt(int pin, void (*ISR)(void), GPIOIntMode mode){
    if(pin == 2){
        // Set the PIN Interrupt type
        xGPIOSPinIntEnable(sD2, mode);
        // Install the interrupt callback function which will be called by EXTI interrupt service function
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD2), xGPIOSPinToPin(sD2), arduPin2_Callback);
        // Enable the GPIO global INT
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD2)));
    } else if(pin == 3){
        xGPIOSPinIntEnable(sD3, mode);
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD3), xGPIOSPinToPin(sD3), arduPin3_Callback);
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD3)));
    }else if(pin == 4){
        xGPIOSPinIntEnable(sD4, mode);
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD4), xGPIOSPinToPin(sD4), arduPin4_Callback);
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD4)));
    }else if(pin == 5){
        xGPIOSPinIntEnable(sD5, mode);
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD5), xGPIOSPinToPin(sD5), arduPin5_Callback);
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD5)));
    }else if(pin == 6){
        xGPIOSPinIntEnable(sD6, mode);
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD6), xGPIOSPinToPin(sD6), arduPin6_Callback);
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD6)));
    }else if(pin == 7){
        xGPIOSPinIntEnable(sD7, mode);
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD7), xGPIOSPinToPin(sD7), arduPin7_Callback);
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD7)));
    }else if(pin == 8){
        xGPIOSPinIntEnable(sD8, mode);
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD8), xGPIOSPinToPin(sD8), arduPin8_Callback);
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD8)));
    }else if(pin == 9){
        xGPIOSPinIntEnable(sD9, mode);
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD9), xGPIOSPinToPin(sD9), arduPin9_Callback);
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD9)));
    }else if(pin == 10){
        xGPIOSPinIntEnable(sD10, mode);
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD10), xGPIOSPinToPin(sD10), arduPin10_Callback);
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD10)));
    }else if(pin == 11){
        xGPIOSPinIntEnable(sD11, mode);
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD11), xGPIOSPinToPin(sD11), arduPin11_Callback);
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD11)));
    }else if(pin == 12){
        xGPIOSPinIntEnable(sD12, mode);
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD12), xGPIOSPinToPin(sD12), arduPin12_Callback);
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD12)));
    }else if(pin == 13){
        xGPIOSPinIntEnable(sD13, mode);
        xGPIOPinIntCallbackInit(xGPIOSPinToPort(sD13), xGPIOSPinToPin(sD13), arduPin13_Callback);
        xIntEnable(xSysCtlPeripheraIntNumGet(xGPIOSPinToPort(sD13)));
    }
    arduPinIntFun[pin] = ISR;
}

void detachInterrupt(int pin){
    // Set the PIN Interrupt type
    switch(pin){
        case 2:  xGPIOSPinIntDisable(sD2); break;
        case 3:  xGPIOSPinIntDisable(sD3); break;
        case 4:  xGPIOSPinIntDisable(sD4); break;
        case 5:  xGPIOSPinIntDisable(sD5); break;
        case 6:  xGPIOSPinIntDisable(sD6); break;
        case 7:  xGPIOSPinIntDisable(sD7); break;
        case 8:  xGPIOSPinIntDisable(sD8); break;
        case 9:  xGPIOSPinIntDisable(sD9); break;
        case 10: xGPIOSPinIntDisable(sD10); break;
        case 11: xGPIOSPinIntDisable(sD11); break;
        case 12: xGPIOSPinIntDisable(sD12); break;
        case 13: xGPIOSPinIntDisable(sD13); break;
    }
}

/*
 *@brief: Re-maps a number from one range to another. That is, a value of fromLow would get mapped
          to toLow, a value of fromHigh to toHigh, values in-between to values in-between, etc. 
 *@param: value the number to map
 *@param: fromLow the lower bound of the value's current range
 *@param: fromHigh the upper bound of the value's current range
 *@param: toLow the lower bound of the value's target range
 *@param: toHigh the upper bound of the value's target range 
 *@return: The mapped value. 
 */
long map(long value, long in_min, long in_max, long out_min, long out_max)
{ 
     return (value - in_min)*(out_max - out_min) / (in_max - in_min)+out_min;
} 

/* 
 *@brief: Initializes the pseudo-random number generator, causing it to start at an arbitrary point  
          in its random sequence. This sequence, while very long, and random, is always the same. 
 *@param: seed pass a number to generate the seed.
 *@return: None.
 */
void randomSeed(unsigned int seed)
{
    if (seed != 0){
        srand(seed);
    }
}

/* 
 *@brief: The random function generates pseudo-random numbers.
 *@param: howbig upper bound of the random value, exclusive.
 *@return: a random number between 0 and max-1 (long)
 */
long random(long howbig)
{
    if (howbig == 0) {
        return 0;
    }
    return rand()%howbig;
} 

/* 
 *@brief: The random function generates pseudo-random numbers.
 *@param: min lower bound of the random value, inclusive (optional)  
 *@param: max upper bound of the random value, exclusive.
 *@return: a random number between min and max-1 (long)
 */
long random(long min, long max)
{
     if (min >= max){
          return min;
     }
     long diff = max - min;
     return random(diff) + min;
} 

unsigned int makeWord(unsigned int w) {
    return w;
}

unsigned int makeWord(unsigned char h, unsigned char l) {
    return (h << 8) | l;
}

/*
 *@bref: Optimized CRC-XMODEM calculation.
         Polymomial: x^16 + x^12 + x^5 + 1 (0x1021)
         Initial value: 0x0
*/
uint16_t inline _crc_xmodem_update(uint16_t crc, uint8_t data){
    int i;
    crc = crc ^ ((uint16_t)data << 8);
    for(i = 0; i < 8; i++){
      if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
    return crc;
}

/*
 *@bref: Optimized CRC-XMODEM calculation.
         Polymomial: x^16 + x^12 + x^5 + 1 (0x8408)
         Initial value: 0xffff
         This is the CRC used by PPP and IrDA.
*/
#define lo8(x) (x&&0xff)
#define hi8(x) ((x&&0xff00)>>8)
uint16_t inline _crc_ccitt_update(uint16_t crc, uint8_t data){
    data ^= lo8(crc);
    data ^= data << 4;

    return((((uint16_t)data << 8) | hi8(crc)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

uint8_t inline _crc_ibutton_update(uint8_t crc, uint8_t data) {
    crc = crc ^ data;
    for (uint8_t i = 0; i < 8; i++){
        if (crc & 0x01)  crc = (crc >> 1) ^ 0x8C;
        else  crc >>= 1;
    }
    return crc;
}

uint16_t inline _crc16_update(uint16_t crc, uint8_t data){
    crc ^= data;
    for (int i = 0; i < 8; ++i){
        if (crc & 1)  crc = (crc >> 1) ^ 0xA001;
        else  crc = (crc >> 1);
    }
    return crc;
}

uint16_t crc_xmodem_check (uint8_t *data , uint16_t size){
    uint16 crc=0;
    for(uint16_t i=0;i<size;i++) crc = _crc_xmodem_update(crc, data[i]);
    return crc;
}

uint8_t crc_ibutton_checck(uint8_t *data , uint16_t size){
    uint8_t crc=0;
    for(uint16_t i=0;i<size;i++) crc = _crc_ibutton_update(crc, data[i]);
    return crc;
}

uint16_t crc16_check (uint8_t *data , uint16_t size){
    uint16_t crc=0xffff;
    for(uint16_t i=0;i<size;i++) crc = _crc16_update(crc, data[i]);
    return crc;
}

uint16_t crc_ccitt_check(uint8_t *data , uint16_t size){
    uint16_t crc=0xffff;
    for(uint16_t i=0;i<size;i++) crc = _crc_ccitt_update(crc, data[i]);
    return crc;
}

System SYS = System();
