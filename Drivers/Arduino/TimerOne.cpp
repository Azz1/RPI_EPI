/*
 *  Interrupt and PWM utilities for 16 bit Timer1 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by Jérôme Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified June 2011 by Lex Talionis to add a function to read the timer
 *  Modified Oct 2011 by Andrew Richards to avoid certain problems:
 *  - Add (long) assignments and casts to TimerOne::read() to ensure calculations involving tmp, ICR1 and TCNT1 aren't truncated
 *  - Ensure 16 bit registers accesses are atomic - run with interrupts disabled when accessing
 *  - Remove global enable of interrupts (sei())- could be running within an interrupt routine)
 *  - Disable interrupts whilst TCTN1 == 0.  Datasheet vague on this, but experiment shows that overflow interrupt 
 *    flag gets set whilst TCNT1 == 0, resulting in a phantom interrupt.  Could just set to 1, but gets inaccurate
 *    at very short durations
 *  - startBottom() added to start counter at 0 and handle all interrupt enabling.
 *  - start() amended to enable interrupts
 *  - restart() amended to point at startBottom()
 * Modiied 7:26 PM Sunday, October 09, 2011 by Lex Talionis
 *  - renamed start() to resume() to reflect it's actual role
 *  - renamed startBottom() to start(). This breaks some old code that expects start to continue counting where it left off
 *
 *  This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  See Google Code project http://code.google.com/p/arduino-timerone/ for latest
 */

#include "Arduino.h"
#include "TimerOne.h"

TimerOne Timer1;              // preinstatiate

// Timer4 interrupt callback function, to deal with step motor
static unsigned long TimerCallback(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData)
{
    // Clear the status
    xTimerStatueClear(sTIMER_BASE, xTIMER_CHANNEL0, xTIMER_INT_MATCH);
    Timer1.isrCallback();
    return 0;
}

void TimerOne::initialize(long microseconds)
{
    xSysCtlPeripheralEnable2(sTIMER_BASE);
 
    // Clear the flag first
    xTimerStatueClear(sTIMER_BASE, xTIMER_CHANNEL0, xTIMER_INT_MATCH);

    // Config timer4 as periodic mode, 1000Hz interrupt frequency
    // This timer as a trigger source for step motor and other peripheral components
    xTimerInitConfig(sTIMER_BASE, xTIMER_CHANNEL0, xTIMER_MODE_PERIODIC | xTIMER_COUNT_UP, 1000000/microseconds);
}


void TimerOne::setPeriod(long microseconds)		// AR modified for atomic access
{
    xIntDisable(sTIMER_BASE);
    xTimerInitConfig(sTIMER_BASE, xTIMER_CHANNEL0, xTIMER_MODE_PERIODIC | xTIMER_COUNT_UP, 1000000/microseconds);
    xIntEnable(sTIMER_BASE);
}

void TimerOne::setPwmDuty(char pin, int duty)
{
  
}

void TimerOne::pwm(char pin, int duty, long microseconds)  // expects duty cycle to be 10 bit (1024)
{

}

void TimerOne::disablePwm(char pin)
{

}

void TimerOne::attachInterrupt(void (*isr)(), long microseconds)
{
    xIntDisable(sTIMER_BASE);
    xTimerInitConfig(sTIMER_BASE, xTIMER_CHANNEL0, xTIMER_MODE_PERIODIC | xTIMER_COUNT_UP, 1000000/microseconds);
    xIntEnable(sTIMER_BASE);
	
    xTimerIntCallbackInit(sTIMER_BASE, TimerCallback);
    xTimerIntEnable(sTIMER_BASE, xTIMER_CHANNEL0, xTIMER_INT_MATCH);
    xIntPrioritySet(sTIMER_BASE, 20);  //a low priority
    xIntEnable(sTIMER_BASE);
}

void TimerOne::detachInterrupt()
{
    xTimerIntDisable(sTIMER_BASE, xTIMER_CHANNEL0, xTIMER_INT_MATCH);
    xIntDisable(sTIMER_BASE);
}

void TimerOne::resume()				// AR suggested
{ 

}

void TimerOne::restart()		// Depricated - Public interface to start at zero - Lex 10/9/2011
{
    start();				
}

void TimerOne::start()	// AR addition, renamed by Lex to reflect it's actual role
{
    xTimerStop(sTIMER_BASE, xTIMER_CHANNEL0);
    xTimerLoadSet(sTIMER_BASE, xTIMER_CHANNEL0, 0);
    xTimerStart(sTIMER_BASE, xTIMER_CHANNEL0);
}

void TimerOne::stop()
{
    xTimerStop(sTIMER_BASE, xTIMER_CHANNEL0);
}

unsigned long TimerOne::read()		//returns the value of the timer in microseconds
{									//rember! phase and freq correct mode counts up to then down again
    return xTimerValueGet(sTIMER_BASE, xTIMER_CHANNEL0);
}
