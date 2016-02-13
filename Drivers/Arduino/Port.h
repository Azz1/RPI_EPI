//*****************************************************************************
//
//! \file Port.h
//! \brief Defines and Macros for STM32F103 Nucleo Board.
//! \version V1.0.0.0
//! \date 4/20/2014
//! \author CooCox
//! \copy
//!
//! Copyright (c)  2014, CooCox
//! All rights reserved.
//! 
//! Redistribution and use in source and binary forms, with or without 
//! modification, are permitted provided that the following conditions 
//! are met: 
//! 
//!     * Redistributions of source code must retain the above copyright 
//! notice, this list of conditions and the following disclaimer. 
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution. 
//!     * Neither the name of the <ORGANIZATION> nor the names of its 
//! contributors may be used to endorse or promote products derived 
//! from this software without specific prior written permission. 
//! 
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#ifndef __STM32F1_NUCLEO_H__
#define __STM32F1_NUCLEO_H__

#include "xhw_types.h"
#include "xhw_memmap.h"
#include "xhw_ints.h"
#include "xhw_nvic.h"
#include "xdebug.h"
#include "xcore.h"
#include "xhw_sysctl.h"
#include "xsysctl.h"
#include "xhw_gpio.h"
#include "xgpio.h"
#include "xhw_uart.h"
#include "xuart.h"
#include "xhw_spi.h"
#include "xspi.h"
#include "xhw_i2c.h"
#include "xi2c.h"
#include "xpwm.h"
#include "xhw_adc.h"
#include "xadc.h"
#include "xhw_timer.h"
#include "xtimer.h"

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup STM32F1_NUCLEO_Board STM32F1_NUCLEO_Board
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup STM32F1_NUCLEO_Board_Res STM32F1_NUCLEO Board Resource
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup STM32F1_NUCLEO_Arduino_Compatible STM32F1_NUCLEO Arduino Compatible Part
//! @{
//
//*****************************************************************************

#define PinToPWMBase(pin)         (pin > 3) ? xPWMB_BASE : xPWMA_BASE

//*****************************************************************************
//
//! \addtogroup STM32F1_NUCLEO_Arduino_Digital STM32F1_NUCLEO Arduino Compatible Digital Part
//! @{
//
//*****************************************************************************

#define sD0                     PA10
								//PA3
#define sD1                     PA9
								//PA2
#define sD2                     PA10
#define sD3                     PB3
#define sD4                     PB5
#define sD5                     PB4
#define sD6                     PB10
#define sD7                     PA8
#define sD8                     PA9
#define sD9                     PC7
#define sD10                    PB6
#define sD11                    PA7
#define sD12                    PA6
#define sD13                    PA5
#define sD14                    PB9
#define sD15                    PB8

#define sSDA                    PB11
								//PB9
#define sSCL                    PB10
								//PB8

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup STM32F1_NUCLEO_Arduino_ANALOG STM32F1_NUCLEO Arduino Compatible ANALOG Part
//! @{
//
//*****************************************************************************

#define sA0                     PA0
#define sA1                     PA1
#define sA2                     PA4
#define sA3                     PB0
#define sA4                     PC1
#define sA5                     PC0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup STM32F1_NUCLEO_Arduino_ICSP STM32F1_NUCLEO Arduino Compatible ICSP Part
//! @{
//
//*****************************************************************************

#define sICSP_1_MISO            PA6
#define sICSP_3_SCK             PA5
#define sICSP_4_MOSI            PA7

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup STM32F1_NUCLEO_Arduino_Communication STM32F1_NUCLEO Arduino Compatible Communication Part
//! @{
//
//*****************************************************************************

#define sSPI_BASE               xSPI1_BASE
#define sICSP_SPI_BASE          xSPI1_BASE
#define sI2C_BASE               xI2C2_BASE
#define sADC_BASE               xADC1_BASE
#define sUART_BASE              xUART1_BASE
#define sPWMA_BASE              xPWMA_BASE
#define sPWMB_BASE              xPWMD_BASE
#define sTIMER_BASE             xTIMER4_BASE

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup STM32F1_NUCLEO_Exported_APIs STM32F103 NUCLEO API
//! \brief STM32F103 NUCLEO Pin Configuration API
//!
//! @{
//
//*****************************************************************************
#define sPinTypeUART(ulBase)                                                  \
        do                                                                    \
        {                                                                     \
           xSPinTypeUART(UART1RX, sD0);                                       \
           xSPinTypeUART(UART1TX, sD1);                                       \
        }while(0)

#define sPinTypeSPI(ulBase)                                                   \
        do                                                                    \
        {                                                                     \
           xSPinTypeSPI(SPI1MOSI(APP),sD11);                                  \
           xSPinTypeSPI(SPI1MISO(PIN),sD12);                                  \
           xSPinTypeSPI(SPI1CLK(APP), sD13);                                  \
        }while(0)


#define sPinTypeI2C(ulBase)                                                   \
        do                                                                    \
        {                                                                     \
           xSPinTypeI2C(I2C1SDA, sD14);                                       \
           xSPinTypeI2C(I2C1SCK, sD15);                                       \
        }while(0)


#define sA0PinTypeADC()         xSPinTypeADC(ADC0, sA0)
#define sA1PinTypeADC()         xSPinTypeADC(ADC1, sA1)
#define sA2PinTypeADC()         xSPinTypeADC(ADC4, sA2)
#define sA3PinTypeADC()         xSPinTypeADC(ADC8, sA3)
#define sA4PinTypeADC()         xSPinTypeADC(ADC11, sA4)
#define sA5PinTypeADC()         xSPinTypeADC(ADC10, sA5)

#define sD3PinTypePWM()         xSPinTypeTimer(TIM2CH2(APP), sD3)
#define sD5PinTypePWM()         xSPinTypeTimer(TIM3CH1(APP), sD5)
#define sD6PinTypePWM()         xSPinTypeTimer(TIM2CH3(APP), sD6)
#define sD9PinTypePWM()         xSPinTypeTimer(TIM3CH2(APP), sD9)
#define sD10PinTypePWM()        xSPinTypeTimer(TIM4CH1(APP), sD10)
#define sD11PinTypePWM()        xSPinTypeTimer(TIM14CH1(APP), sD11)


//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __STM32F1_NUCLEO_H__
