/*
ServoLibrary.c

Created: 11/3/2017 7:22:26 PM
Copyright (c) 2018-2019, Martin Sant
All rights reserved.

Redistribution and use in source and binary forms, with or
without modification, are permitted provided that the following
conditions are met: 
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
*/ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include "servotimer.h" 


 // servo states
 #define SERVOIDLE    00
 #define STARTSERVO0  11
 #define WAITSERVO0   12
 #define STARTSERVO1  14
 #define WAITSERVO1   15
 #define STARTSERVO2  17
 #define WAITSERVO2   18
 #define ENDSERVO     22

 #define ONEMS        2000	    /* ONE ms @ 16mhz */
 #define NEXTSCAN     16*ONEMS      /* 16 ms just will fit into 16 bits */
 #define ENDSCAN      10*ONEMS      /* generic time to wait until next servo */
 #define SERVOSOFF    0xf8          /* bottom three pins are the 3 servo outputs */
 
 #define USTIMER      0x01          /* OVF overflow vector - uS timer */
 #define SERVOTIMER   0x02          /* COMPA vector - servos */
 #define COMPAREB     0x04          /* COMPB vector - update accel/decl for servo 0 */

 #define SERVO0       0x01          /* define servo outputs port bits */
 #define SERVO1       0x02
 #define SERVO2       0x04

/* Handle Three Servo outputs on PORTB bits 0-2 */

static volatile int16_t ServoPulseMs0 = ONEMS;      /* Actual values being clocked out: 1000 min, 2000 max */
static volatile int16_t ServoPulseMs1 = ONEMS;
static volatile int16_t ServoPulseMs2 = ONEMS;

static volatile int16_t ServoDefault0 = ONEMS;    /* ADDED to Above for 2000 total */
static volatile int16_t ServoDefault1 = ONEMS;
static volatile int16_t ServoDefault2 = ONEMS;

static volatile int16_t ServoHighLimit0 = ONEMS;    /* ADDED to Above for 2000 total */
static volatile int16_t ServoHighLimit1 = ONEMS;
static volatile int16_t ServoHighLimit2 = ONEMS;

static volatile int16_t ServoLowLimit0 = 0;         /* ADDED to Above for 1000 total */
static volatile int16_t ServoLowLimit1 = 0;
static volatile int16_t ServoLowLimit2 = 0;

static volatile uint8_t servo0Dir = 0;
static volatile uint8_t servo1Dir = 0;
static volatile uint8_t servo2Dir = 0;

static volatile int16_t watchDog = 3000;

 // set up the clock so it runs at 1us per tick

static volatile uint64_t msClock;
static volatile uint16_t msClockHigh;
static volatile uint32_t msUpper;

static volatile uint8_t servostate;


uint16_t normalMs = 2000; // 1 ms in timer updates

/* starts both the servo cycle and the master 1us clock source */

void initServoTimer(void)
{
     msClock     = 0;
     msClockHigh = 0;
     msUpper     = 0;

     DDRC = 0x07;
    
     servostate = SERVOIDLE;
     
     OCR1A = NEXTSCAN;			// This starts the servo scan.
     OCR1B = normalMs;

     TCCR1A = 0;
     TCCR1B = 0;
     TCNT1  = 0;

     TCCR1B |= 0x02;			          // clock select, divide sysclock by 8
     TIMSK1 |= USTIMER | SERVOTIMER | COMPAREB;   // enable interrupts 

     // Overflow and OCR interrupts, let timer run until overflow, keep track of upper word in s/w 
}

/* Master Clock Source                                  */
/* ------------------------ 64 bits of microseconds --- */
/*   msUpper 32bits   MsClockHigh 16bits  TCLK 16bits   */
/*   0000 0000        0000                0000          */
/*                                                      */
/*   292471 years before it rolls over                  */

ISR(TIMER1_OVF_vect)
{
    msClockHigh++;
    if(msClockHigh == 0)
       msUpper++;
}


ISR(TIMER1_COMPB_vect)
{
    watchDog = watchDog - 1;
    if (watchDog < 0)
        watchDog = 0;

    OCR1B = normalMs + TCNT1;     // one ms from where we are

}

/* handle each servo one at a time */

ISR(TIMER1_COMPA_vect)
{
    switch(servostate)
    {
        case SERVOIDLE:
        PORTC &= SERVOSOFF;
        OCR1A += ONEMS;
        servostate = STARTSERVO0;
        break;
        
        case STARTSERVO0:
        PORTC |= SERVO0;
        OCR1A += ServoPulseMs0;
        servostate = WAITSERVO0;
        break;

        case WAITSERVO0:
        PORTC &= ~(SERVO0);
        OCR1A += ONEMS;
        servostate = STARTSERVO1;
        break;
        
        case STARTSERVO1:
        PORTC |= SERVO1;
        OCR1A += ServoPulseMs1;
        servostate = WAITSERVO1;
        break;
        
        case WAITSERVO1:
        PORTC &= ~(SERVO1);
        OCR1A += ONEMS;
        servostate = STARTSERVO2;
        break;

        case STARTSERVO2:
        PORTC |= SERVO2;
        OCR1A += ServoPulseMs2;
        servostate = ENDSERVO;
        break;

        case ENDSERVO:
        PORTC &= ~(SERVO2);
        OCR1A += ENDSCAN;
        servostate = SERVOIDLE;
        break;
    }
}

int16_t getWatchDog()
{
   return watchDog;
}

void resetWatchDog(int16_t value)
{
   watchDog = value;
}


uint64_t scratch;

uint64_t getMsClock()
{
    scratch = msUpper;
    scratch = (scratch << 32);

    msClock = msClockHigh;
    msClock = (msClock << 16);
    msClock |= TCNT1;
    msClock |= scratch;
    return msClock;
}

void delay_us(uint32_t t)
{
     uint64_t now;
     uint64_t del;

     del = now = getMsClock();
     
     while( (now - del) < t )
     now = getMsClock();
}


void setServoPulse(uint8_t i, int16_t pulse)
{
    /* Global sanity check pulse value */

    if (pulse < 0 )
    return;

    if (pulse > 1000 )
    return;

    switch(i)
    {
        case 0: if(pulse < ServoLowLimit0)  pulse = ServoLowLimit0;
                if(pulse > ServoHighLimit0) pulse = ServoHighLimit0;
                if(servo0Dir) pulse = 1000 - pulse;
                pulse = pulse * 2;    // 16Mhz, double the time
                ServoPulseMs0 = pulse + ONEMS;
        break;

        case 1: if(pulse < ServoLowLimit1)  pulse = ServoLowLimit1;
                if(pulse > ServoHighLimit1) pulse = ServoHighLimit1;
				if(servo1Dir) pulse = 1000 - pulse;
                pulse = pulse * 2;
                ServoPulseMs1 = pulse + ONEMS;
        break;

        case 2: if(pulse < ServoLowLimit2)  pulse = ServoLowLimit2;
                if(pulse > ServoHighLimit2) pulse = ServoHighLimit2;
				if(servo2Dir) pulse = 1000 - pulse;
				pulse = pulse * 2;    // 16Mhz, double the time
                ServoPulseMs2 = pulse + ONEMS;
        break;

    }
}

void setServoDefault0(int16_t sd)
{
  if((sd >= 0) && (sd <= 1000))
     ServoDefault0 = sd;
}
void setServoDefault1(int16_t sd)
{
	if((sd >= 0) && (sd <= 1000))
	ServoDefault1 = sd;
}
void setServoDefault2(int16_t sd)
{
	if((sd >= 0) && (sd <= 1000))
	ServoDefault2 = sd;
}


void setServoLow0(int16_t lo)
{
	if((lo >= 0) && (lo <= 1000))
	ServoLowLimit0 = lo;
}
void setServoLow1(int16_t lo)
{
	if((lo >= 0) && (lo <= 1000))
	ServoLowLimit1 = lo;
}
void setServoLow2(int16_t lo)
{
	if((lo >= 0) && (lo <= 1000))
	ServoLowLimit2 = lo;
}


void setServoHigh0(int16_t hi)
{
	if((hi >= 0) && (hi <= 1000))
	ServoHighLimit0 = hi;
}

void setServoHigh1(int16_t hi)
{
	if((hi >= 0) && (hi <= 1000))
	ServoHighLimit1 = hi;
}
void setServoHigh2(int16_t hi)
{
	if((hi >= 0) && (hi <= 1000))
	ServoHighLimit2 = hi;
}


void setServoReverseValue(uint8_t direction)
{
  servo0Dir = direction & 1;
  servo1Dir = direction & 2;
  servo2Dir = direction & 4;
}


