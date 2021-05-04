/*
uart.c

Created: 7/14/2016 7:24:58 PM
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
#include "uart.h"

// Init usart on atmega328, transmit for debug

void initUART(long baud)
{
    long   baud_prescale = (F_CPU / (baud * 16)) - 1;	// from the data sheet
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);		// enable tx and rx
    UCSR0C = (1 << USBS0) | (3 << UCSZ00);		// 8 bits, 1 stop, no parity
    UBRR0L = baud_prescale;				// baud rate
    UBRR0H = (baud_prescale >> 8);
}

// Transmit a byte, this is a blocking routine, it waits on the tx to be empty
void SendByte(uint8_t c)
{
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}


