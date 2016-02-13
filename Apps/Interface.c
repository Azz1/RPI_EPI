#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "xhw_types.h"
#include "xcore.h"
#include "xhw_ints.h"
#include "xhw_memmap.h"
#include "xhw_nvic.h"
#include "xhw_sysctl.h"
#include "xhw_gpio.h"
#include "xhw_timer.h"
#include "xdebug.h"
#include "xsysctl.h"
#include "xgpio.h"
#include "cookie.h"
#include "xadc.h"
#include "xtimer.h"
#include "DriverCfg.h"
//#include "Adafruit_MotorShield.h"

#define STAT_MOTOR_UNINIT    0
#define STAT_MOTOR_INIT      1
#define STAT_MOTOR_OVER      5

//
// Timer4 interrupt callback function, to deal with step motor
//
unsigned long Timer4Callback(void *pvCBData,  unsigned long ulEvent,
                                       unsigned long ulMsgParam,
                                       void *pvMsgData)
{
    return 0;
}

unsigned long MotorShieldOperation(CMDSTRUCT *pCmdStruct)
{
    //
    // MotoShield initialization status.
    //
    static unsigned char ucStatus = STAT_MOTOR_UNINIT;
    unsigned long ulTmp = 0;
    unsigned char i;
	unsigned char ucDir, ucStyle;
	unsigned long ulTotalStep;


    if(pCmdStruct->cmd_1 == 0x40 && pCmdStruct->cmd_2 == 0x0) //Èç¹ûÊÇmotorshield
    {
        switch(pCmdStruct->cmd_3)
        {

        case 0x05:  // Step motor initialize
        	Adafruit_MotorShield_Init();

            ucStatus = STAT_MOTOR_INIT;
            pCmdStruct->ret = TRUE;
            pCmdStruct->tx_len = 0;
            break;

        case 0x06:  // Step motor speed set
            ulTmp = *(unsigned long *)&pCmdStruct->rx_buf[1];

            if(pCmdStruct->rx_buf[0] == 0x00) {
            	Adafruit_MotorShield_SetSpeed(0, ulTmp);
        	} else {
            	Adafruit_MotorShield_SetSpeed(1, ulTmp);
        	}

            pCmdStruct->ret = TRUE;
            pCmdStruct->tx_len = 0;
            break;

        case 0x07:  // Step motor start
        	ucDir = pCmdStruct->rx_buf[1];
        	ucStyle = pCmdStruct->rx_buf[2];
        	ulTotalStep = *(unsigned long *)&pCmdStruct->rx_buf[3];

        	if(pCmdStruct->rx_buf[0] == 0x00) {
        		Adafruit_MotorShield_Step(0, ulTotalStep, ucDir, ucStyle);
        	} else {
        		Adafruit_MotorShield_Step(1, ulTotalStep, ucDir, ucStyle);
        	}

            break;

        case 0x08:  // Step motor stop

        	if(pCmdStruct->rx_buf[0] == 0x00) {
            	Adafruit_MotorShield_Release(0);
        	} else {
            	Adafruit_MotorShield_Release(1);
        	}

            pCmdStruct->ret = TRUE;
            pCmdStruct->tx_len = 0;
            break;

        default :
            pCmdStruct->ret = FALSE;
            pCmdStruct->tx_len = 0;
            break;
        }
    }
	return 0;
}

unsigned long UltrasonicSensorOperation(CMDSTRUCT *pCmdStruct)
{
    unsigned short tmp = 0;
    unsigned char *p = (unsigned char *)&tmp;
    if((pCmdStruct->cmd_1 == 0x40)&&(pCmdStruct->cmd_2 == 0x02))  //Ultrasonic
    {
        switch(pCmdStruct->cmd_3)
        {
            case 0x01:
                HCSR04Init();
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 0;
                break;
            case 0x02:
                //*(unsigned short *)pCmdStruct->tx_buf = HCSR04GetDistance();
                //*(unsigned short *)pCmdStruct->tx_buf = HCSR04GetDistance();
                tmp = HCSR04GetDistance();
                //printf("%d\r\n", tmp);
                pCmdStruct->tx_buf[0] = *p++;
                pCmdStruct->tx_buf[1] = *p++;

                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 2;
                break;
            case 0x03:
                HCSR04Stop();
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 0;
                break;
            default:
                pCmdStruct->ret = FALSE;
                pCmdStruct->tx_len = 0;
                break;
        }
    }

    return (0);
}

//*****************************************************************************
//
//! \brief TinkerKit operation
//!
//! \param pCmdStruct the command struct to pass parameter and get result
//!
//! This function is to initialize the IO port and timer to get ready for measure
//!
//! \return None.
//
//*****************************************************************************
unsigned long TinkerKitOperation(CMDSTRUCT *pCmdStruct)
{
    unsigned long ulADCTmp[4];
    unsigned char i;

    if((pCmdStruct->cmd_1 == 0x40)&&(pCmdStruct->cmd_2 == 0x1))  //tinkerkit
    {
        switch(pCmdStruct->cmd_3)
        {
        case 0x01:
            switch(pCmdStruct->rx_buf[0])
            {
            case 0: // O0 -> sD11
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sD11));
                xGPIOSPinTypeGPIOOutput(sD11);
                xGPIOSPinWrite(sD11, pCmdStruct->rx_buf[1]);
                pCmdStruct->ret = TRUE;
                break;
            case 1: // O1 -> sD10
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sD10));
                xGPIOSPinTypeGPIOOutput(sD10);
                xGPIOSPinWrite(sD10, pCmdStruct->rx_buf[1]);
                pCmdStruct->ret = TRUE;
                break;
            case 2: // O2 -> sD9
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sD9));
                xGPIOSPinTypeGPIOOutput(sD9);
                xGPIOSPinWrite(sD9, pCmdStruct->rx_buf[1]);
                pCmdStruct->ret = TRUE;
                break;
            case 3: // O3 -> sD6
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sD6));
                xGPIOSPinTypeGPIOOutput(sD6);
                xGPIOSPinWrite(sD6, pCmdStruct->rx_buf[1]);
                pCmdStruct->ret = TRUE;
                break;
            case 4: // O4 -> sD5
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sD5));
                xGPIOSPinTypeGPIOOutput(sD5);
                xGPIOSPinWrite(sD5, pCmdStruct->rx_buf[1]);
                pCmdStruct->ret = TRUE;
                break;
            case 5: // O5 -> sD3
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sD3));
                xGPIOSPinTypeGPIOOutput(sD3);
                xGPIOSPinWrite(sD3, pCmdStruct->rx_buf[1]);
                pCmdStruct->ret = TRUE;
                break;
            default:
                pCmdStruct->ret = FALSE;
                break;
            }
            pCmdStruct->tx_len = 0;
            break;
        case 0x02:
            switch(pCmdStruct->rx_buf[0])
            {
            case 0: // I0 -> sA0
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA0));
                xGPIOSPinTypeGPIOInput(sA0);
                pCmdStruct->tx_buf[0] =
                        xGPIOSPinRead(sA0);
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 1;
                break;
            case 1: // I1 -> sA1
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA1));
                xGPIOSPinTypeGPIOInput(sA1);
                pCmdStruct->tx_buf[0] =
                        xGPIOSPinRead(sA1);
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 1;
                break;
            case 2: // I2 -> sA2
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA2));
                xGPIOSPinTypeGPIOInput(sA2);
                pCmdStruct->tx_buf[0] =
                        xGPIOSPinRead(sA3);
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 1;
                break;
            case 3: // I3 -> sA3
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA3));
                xGPIOSPinTypeGPIOInput(sA3);
                pCmdStruct->tx_buf[0] =
                        xGPIOSPinRead(sA3);
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 1;
                break;
            case 4: // I4 -> sA4
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA4));
                xGPIOSPinTypeGPIOInput(sA4);
                pCmdStruct->tx_buf[0] =
                        xGPIOSPinRead(sA4);
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 1;
                break;
            case 5: // I5 -> sA5
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA5));
                xGPIOSPinTypeGPIOInput(sA5);
                pCmdStruct->tx_buf[0] =
                        xGPIOSPinRead(sA5);
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 1;
                break;
            default:
                pCmdStruct->ret = FALSE;
                pCmdStruct->tx_len = 0;
                break;
            }
            break;
        case 0x03:

            //
            // Set ADCCLK prescaler, ADCCLK=PCLK2(max 72MHz)/DIV(DIV:2,4,6,8)
            // You should set ADCCLK < 14MHz to ensure the accuracy of ADC
            //
            xSysCtlPeripheralClockSourceSet(xSYSCTL_ADC0_MAIN, 8);
            //
            // Enable Peripheral ADC clock
            //
            xSysCtlPeripheralEnable(xSYSCTL_PERIPH_ADC1);

            //
            // Enable the ADC conversion
            //
            xADCEnable(sADC_BASE);

            //
            // The two sentences below configure ADC to scan mode, continuous convert, software trigger.
            //
            xADCConfigure(sADC_BASE, xADC_MODE_SCAN_SINGLE_CYCLE, ADC_TRIGGER_PROCESSOR);
            xADCConfigure(sADC_BASE, xADC_MODE_SCAN_CONTINUOUS, ADC_TRIGGER_PROCESSOR);

            //
            // Configure channel step by step.(Max 4 steps, the 2nd parameter start from 0, max is 3)
            // Must not jump over a step, or the ADC result may be in wrong position.
            // Here we enable 4 channels of the I0~I3 which work in continuous conversion.
            // You can read one channel a time or multi-channel a time.
            //
            xADCStepConfigure(sADC_BASE, 0, ADC_CHANNEL_10);
            xADCStepConfigure(sADC_BASE, 1, ADC_CHANNEL_11);
            xADCStepConfigure(sADC_BASE, 2, ADC_CHANNEL_12);
            xADCStepConfigure(sADC_BASE, 3, ADC_CHANNEL_13);

            switch(pCmdStruct->rx_buf[0])
            {
            case 0:       // I0 -> sA0
                // Enable IO clock
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA0));
                // Set IO direction and type.
                sA0PinTypeADC();
                // Read ADC value
                xADCDataGet(sADC_BASE, ulADCTmp);
                for(i=0;i<4;i++)
                    pCmdStruct->tx_buf[i] = ulADCTmp[0]>>(i*8);
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 4;
                break;

            case 1:       // I1 -> sA1
                // Enable IO clock
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA1));
                // Set IO direction and type.
                sA1PinTypeADC();
                // Read ADC value
                xADCDataGet(sADC_BASE, ulADCTmp);
                for(i=0;i<4;i++)
                    pCmdStruct->tx_buf[i] = ulADCTmp[1]>>(i*8);
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 4;
                break;
            case 2:       // I2 -> sA2
                // Enable IO clock
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA2));
                // Set IO direction and type.
                sA2PinTypeADC();
                // Read ADC value
                xADCDataGet(sADC_BASE, ulADCTmp);
                for(i=0;i<4;i++)
                    pCmdStruct->tx_buf[i] = ulADCTmp[2]>>(i*8);
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 4;
                break;
            case 3:       // I3 -> sA3
                // Enable IO clock
                xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(sA3));
                // Set IO direction and type.
                sA3PinTypeADC();
                // Read ADC value
                xADCDataGet(sADC_BASE, ulADCTmp);
                for(i=0;i<4;i++)
                    pCmdStruct->tx_buf[i] = ulADCTmp[3]>>(i*8);
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 4;
                break;
            default:
                pCmdStruct->ret = FALSE;
                pCmdStruct->tx_len = 0;
                break;
            }
            break;
        default:
            //
            // If new commands are added
            //
            pCmdStruct->ret = FALSE;
            pCmdStruct->tx_len = 0;
            break;
        }
    }
    return 0;
}

unsigned long IOOperation(CMDSTRUCT *pCmdStruct)
{
    return 0;
}

unsigned long SystemCtl(CMDSTRUCT *pCmdStruct)
{
    unsigned char *FwVer = "STM32 Firmware V1.0.0.0";
    unsigned char *ErrorInfo = "This Command is not supported by this firmware";
    unsigned char index = 0;

    if(pCmdStruct->cmd_3 == 0x02)
    {
        while(*FwVer)
        {
            pCmdStruct->tx_buf[index++] = *FwVer++;
        }
        pCmdStruct->ret = TRUE;
        pCmdStruct->tx_len = index;
    }
    else if(pCmdStruct->cmd_3 == 0x03)
    {
        xSysCtlReset();
    }
    else
    {
        while(*ErrorInfo)
        {
            pCmdStruct->tx_buf[index++] = *ErrorInfo++;
        }
        pCmdStruct->ret = FALSE;
        pCmdStruct->tx_len = index;
    }
    return 0;
}



unsigned long DigitalCompass(CMDSTRUCT *pCmdStruct)
{
    unsigned long angle = 0;
    unsigned char * p = (unsigned char *) &angle;

    int16_t Com_Data[3] = {0};
    Result  Res         = SUCCESS;

    if((pCmdStruct->cmd_1 == 0x40)&&(pCmdStruct->cmd_2 == 0x04))
    {
        switch(pCmdStruct->cmd_3)
        {
            case 0x01: // Init

                //Init Compass
                xSysCtlPeripheralReset2(HMC5883L_PIN_I2C_PORT);
                xSysCtlDelay(0xFFF);
                Res = HMC5883L_Init();
                if(Res == FAILURE)
                {
                    pCmdStruct->ret = FALSE;
                    pCmdStruct->tx_len = 0;
                    break;
                }

                /*
                //Configure Digital Compass
                Res = HMC5883L_Cfg(MODE_CONT | GAIN_1090 | SAMPLE_8 | DATA_RATE_15);
                if(Res == FAILURE)
                {
                    pCmdStruct->ret = FALSE;
                    pCmdStruct->tx_len = 0;
                    break;
                }

                SysCtlDelay(FFF);
                */
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 0;
                break;

            case 0x02: // Read

                //Configure Digital Compass
                Res = HMC5883L_Cfg(MODE_SIG | GAIN_1090 | SAMPLE_8 | DATA_RATE_15);
                if(Res == FAILURE)
                {
                    pCmdStruct->ret = FALSE;
                    pCmdStruct->tx_len = 0;
                    break;
                }

                Res = HMC5883L_DataGet(&Com_Data[0], &Com_Data[1], &Com_Data[2]);
                if(Res == FAILURE)
                {
                    pCmdStruct->ret = FALSE;
                    pCmdStruct->tx_len = 0;
                    break;
                }

                angle = (unsigned long) (atan2((double)Com_Data[0],(double)Com_Data[1])*(180/3.14159265)+180);

                //printf("%f\r\n", angle);
                pCmdStruct->tx_buf[0] = *p++;
                pCmdStruct->tx_buf[1] = *p++;
                pCmdStruct->tx_buf[2] = *p++;
                pCmdStruct->tx_buf[3] = *p++;

                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 4;
                break;
            case 0x03: // Stop
                pCmdStruct->ret = TRUE;
                pCmdStruct->tx_len = 0;
                break;

            default:
                pCmdStruct->ret = FALSE;
                pCmdStruct->tx_len = 0;
                break;
        }
    }

    return (0);
}

