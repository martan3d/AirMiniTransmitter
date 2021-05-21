/*
dcc.cpp

Created: 10/11/2015 8:43:49 AM
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
#include <string.h>
#include "uart.h"
#include "schedule.h"
#include "servotimer.h"
#include "dcc.h"

#define PREAMBLE 0
#define START_BIT 1
#define DATA 2
#define END_BIT 3

// Any variables that are used between the ISR and other functions are declared volatile
unsigned int i;
int16_t usec;
int16_t dnow;
int16_t BitCount;
int8_t  State;
uint8_t dataByte;
uint8_t byteCounter;
uint8_t buffer[sizeof(DCC_MSG)+1];
uint8_t DccBitVal = 0;
uint8_t errorByte = 0;
uint8_t dccbuff[sizeof(DCC_MSG)];
volatile uint16_t transitionCountDCC = 0; // count the number of transitions before a valid state change
volatile uint8_t useModemDataDCC = 1;     // Initial setting for use-of-modem-data state
volatile uint8_t dcLevelDCC = 1;          // The output level (HIGH or LOW) output if modem data is invalid

const uint8_t servotable[] = { 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8, 9, 8, 8, 8, 10, 10, 10 };

uint8_t * getDCC()
{
     return(dccbuff);
}

uint8_t decodeDCCPacket( DCC_MSG * dccptr)
{

   if ((3 <= dccptr->Size) && (dccptr->Size <= 6)) {
      for (uint8_t i = 0; i < dccptr->Size ; i++)
         SendByte(dccptr->Data[i]);
   }
   return dccptr->Size;

}

void dccInit(void)
{
  State  = PREAMBLE;        // Pin change interrupt
  transitionCountDCC = 0;   // Initialize the transition counts before a valid STATE is found
  SET_INPUTPIN;             // Set up a pin for input  (see dcc.h for the actual pin). 
  SET_OUTPUTPIN;            // Set up a pin for output (see dcc.h for the actual pin). 
                            // It will either reproduce D2 or maintain a fixed HIGH/LOW if the modem data is bad.

  // Initially set the output based on dcLevel
  if(dcLevelDCC) OUTPUT_HIGH; // HIGH
  else OUTPUT_LOW;            // LOW

#if defined(TRANSMITTER)
  EICRA  = 0x05;            // Set both EXT0 and EXT1 to trigger on any change
  EIMSK  = 0x02;            // EXT INT 1 enabled only
#else
  EICRA  = 0x05;            // Set both EXT0 and EXT1 to trigger on any change
  EIMSK  = 0x01;            // EXT INT 0 enabled only
#endif
  // Clear the buffer
  memset(buffer,0,sizeof(buffer)); // About the fastest way possible w/o machine code
}

///////////////////////////
// Some access functions //
///////////////////////////
// Access function to get transitionCountDCC
uint16_t getTransitionCount(void) 
{
   return transitionCountDCC;
}

// Access function to reset transitionCountDCC
void resetTransitionCount(uint16_t count) 
{
   transitionCountDCC = count;
}

// Access function to set using modem data state and the DC output value if not using modem data
void DCCuseModemData(uint8_t useModemData_in, uint8_t dcLevel_in)
{
   useModemDataDCC = useModemData_in;
   dcLevelDCC = dcLevel_in;
}
///////////////////////////
///////////////////////////
///////////////////////////

#if defined(TRANSMITTER)
// ExtInterrupt on change of state of port pin D3 on atmega328p, DCC input stream
ISR(INT1_vect)
#else
// ExtInterrupt on change of state of port pin D2 on atmega328p, DCC input stream
ISR(INT0_vect)
#endif
{
#if defined(TRANSMITTER)
    /** PORTD 8 is EXT IRQ 1 - INPUT FOR DCC from optocoupler
	    EXT INT1 gives us an IRQ on both a high and a low change of the input stream */
    if(PIND & 0x08)                         // if it's a one, start of pulse
#else
    /** PORTD 4 is EXT IRQ 0 - INPUT FOR DCC from cc1101 
	    EXT INT0 gives us an IRQ on both a high and a low change of the input stream */
    if(PIND & 0x04)                         // if it's a one, start of pulse
#endif
    {                                       // so, we need to
        usec = TCNT1;                       // snag the current time in microseconds
        if (useModemDataDCC) OUTPUT_HIGH;   // Set D Pin3 HIGH if we are using modem data, otherwise use set DC value
        else 
        {
           if(dcLevelDCC) OUTPUT_HIGH;      // HIGH
           else OUTPUT_LOW;                 // LOW
        }
        return;                             // and that's all we need, exit
    }    
    else                                    // else we are at the end of the pulse, on the downside
    {                                       // how long was the pulse?
        dnow = TCNT1 - usec;                // Get the now time minus the start time gives pulse width in us
        if (useModemDataDCC) OUTPUT_LOW;    // Set D Pin3 LOW if we are using modem data, otherwise use set DC value
        else 
        {
           if(dcLevelDCC) OUTPUT_HIGH;      // HIGH
           else OUTPUT_LOW;                 // LOW
        }
                                            // Longer pulse is a zero, short is one
        if ( dnow > 180 )
            DccBitVal = 0;                  // Longer is a zero
        else 
            DccBitVal = 1;                  // short means a one

        transitionCountDCC++;
    }
    

	/*** after we know if it's a one or a zero, drop through the state machine to see if we are where we think we are in the DCC stream */
    
    BitCount++;                             // Next bit through the state machine

    switch( State )
    {
        case PREAMBLE:                      // preamble is at least 11 bits, but can be more
            if( DccBitVal )
            {
                if( BitCount > 10 )         // once we are sure we have the preamble here, wait on the start bit (low)
                {
                    State = START_BIT;      // Off to the next state
                    transitionCountDCC = 0;    // Reset the transition count
                    memset(buffer,0,sizeof(buffer)); // About the fastest way possible w/o machine code
                }                
            }            
            else
                BitCount = 0 ;              // otherwise sit here and wait on the preamble
        break;
        
        //--------------------------------- Pramble Finished, wait on the start bit

        case START_BIT:                     // preamble at least almost done, wait for the start of the data
             if(!DccBitVal)
             {
                 BitCount = 0;
                 byteCounter = 0;
                 State = DATA;              // soon as we have it, next state, collect the bits for the data bytes
                 transitionCountDCC = 0;       // Reset the transition count
                 dataByte = 0;
             }             
        break;
             
        //---------------------------------- Save the data, clock them into a byte one bit at a time

        case DATA:
            dataByte = (dataByte << 1);
            if(DccBitVal)
                dataByte |= 1;
                
            if(BitCount == 8)
            {
              buffer[byteCounter++] = dataByte;
              dataByte = 0;
              State = END_BIT;
              transitionCountDCC = 0;       // Reset the transition count
            }            
        break;
        
        //--------------------------------- All done, figure out what to do with the data, we now have a complete message

        case END_BIT:
            if( DccBitVal ) // End of packet?
            {
                State = PREAMBLE;                           // Got everything, next time will be start of new DCC packet coming in
                transitionCountDCC = 0;                        // Reset the transition count

                if ((3 <= byteCounter) && ((byteCounter <= 6))) 
                {
                   errorByte  = buffer[0];                 // VERY IMPORTANT!
                   for(uint8_t i = 1; i < byteCounter-1; i++)
                       errorByte ^= buffer[i];                 // All sorts of stuff flies around on the bus
   
                   if (errorByte == buffer[byteCounter-1])
                   {
                       buffer[sizeof(DCC_MSG)-1] = byteCounter;        	// save length
                       buffer[sizeof(DCC_MSG)] = 0;
   
                       for (i=0;i<sizeof(DCC_MSG);i++)     // Move message to buffer for background task
                           dccbuff[i] = buffer[i];
   
                       transitionCountDCC = 0;                // Reset the transition count
                       setScheduledTask(TASK1);            // Schedule the background task
                   }
                }
            }
            else  // Get next Byte
            {
                State = DATA ;
                transitionCountDCC = 0;       // Reset the transition count
            }

            BitCount = 0 ;

        break;

    }
    return;
}

