/*
 * uart.c
 *
 * Created: 7/14/2016 7:24:58 PM
 *  Author: martan
 */ 
 
#include <avr/io.h>
#include "uart.h"

// Init usart on atmega328, transmit for debug

void initUART(long baud)
{
    long   baud_prescale = (F_CPU / (baud * 16)) - 1;	// from the data sheet
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);				// enable tx and rx
    UCSR0C = (1 << USBS0) | (3 << UCSZ00);				// 8 bits, 1 stop, no parity
    UBRR0L = baud_prescale;								// baud rate
    UBRR0H = (baud_prescale >> 8);
}

// Transmit a byte, this is a blocking routine, it waits on the tx to be empty
void SendByte(uint8_t c)
{
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}


