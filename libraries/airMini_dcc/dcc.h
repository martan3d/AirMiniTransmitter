/*
dcc.h

Created: 10/14/2015 6:26:13 PM
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
 
#if defined(__cplusplus)
extern "C" {
#endif

#include <config.h>
#include <avr/io.h>

#ifndef DCC_H_
#define DCC_H_

// Define here the input/output pins

#if defined(TRANSMITTER)
#define INPUT_PIN  PD3  // 5V unipolar DCC input from opto-coupler
#define OUTPUT_PIN PD4  // 5V unipolar DCC diagnostic output, not currently used
#else
#define INPUT_PIN  PD2  // 3.3V unipolar DCC input from raw modem output
#define OUTPUT_PIN PD3  // 5V unipolar DCC filtered output
#endif

#define SET_INPUTPIN  DDRD  &= ~(1<<INPUT_PIN)
#define SET_OUTPUTPIN DDRD  |=  (1<<OUTPUT_PIN)
#define OUTPUT_HIGH   PORTD |=  (1<<OUTPUT_PIN)
#define OUTPUT_LOW    PORTD &= ~(1<<OUTPUT_PIN)

#define DCCMAXLEN 6 // This is the maximum # of bytes of a DCC packet (not 5!)
typedef struct
{
    unsigned char Data[DCCMAXLEN];
    unsigned char Size;
	
} DCC_MSG ;

void dccInit(void);
unsigned char * getDCC();
unsigned char decodeDCCPacket( DCC_MSG * dccptr);
unsigned short getTransitionCount();
void resetTransitionCount(unsigned short count);
void DCCuseModemData(unsigned char useModemData_in,unsigned char muxCVval_in);

#endif /* DCC_H_ */

#if defined(__cplusplus)
}
#endif
