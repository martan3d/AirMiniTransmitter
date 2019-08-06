/*
 * dcc.cpp
 *
 * Created: 10/11/2015 8:43:49 AM
 *  Author: martan
 *
 */

#define TRANSMIT

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
	uint8_t l;
	
	l = dccptr->Size;       // length of packet
	
	switch(l)
	{
		case 3:             // three bytes
		        SendByte(dccptr->Data[0]);
		        SendByte(dccptr->Data[1]);
		        SendByte(dccptr->Data[2]);
		        break;
				
		case 4:
		        SendByte(dccptr->Data[0]);      // debug out the serial port
		        SendByte(dccptr->Data[1]);
		        SendByte(dccptr->Data[2]);
		        SendByte(dccptr->Data[3]);
			break;
				
		case 5:
		        SendByte(dccptr->Data[0]);
		        SendByte(dccptr->Data[1]);
		        SendByte(dccptr->Data[2]);
		        SendByte(dccptr->Data[3]);
		        SendByte(dccptr->Data[4]);
		        break;

		case 6:
		        SendByte(dccptr->Data[0]);
		        SendByte(dccptr->Data[1]);
		        SendByte(dccptr->Data[2]);
		        SendByte(dccptr->Data[3]);
		        SendByte(dccptr->Data[4]);
		        SendByte(dccptr->Data[5]);
		        break;
	}

        return l;
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

#ifdef TRANSMIT
  EICRA  = 0x05;            // Set both EXT0 and EXT1 to trigger on any change
  EIMSK  = 0x02;            // EXT INT 1 enabled only
#else
  EICRA  = 0x05;            // Set both EXT0 and EXT1 to trigger on any change
  EIMSK  = 0x01;            // EXT INT 0 enabled only
#endif
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

#ifdef TRANSMIT
// ExtInterrupt on change of state of port pin D3 on atmega328p, DCC input stream
ISR(INT1_vect)
#else
// ExtInterrupt on change of state of port pin D2 on atmega328p, DCC input stream
ISR(INT0_vect)
#endif
{
#ifdef TRANSMIT
    /** PORTD 8 is EXT IRQ 1 - INPUT FOR DCC from optocoupler
	    EXT INT1 gives us an IRQ on both a high and a low change of the input stream */
    if(PIND & 0x08)                         // if it's a one, start of pulse
#else
    /** PORTD 4 is EXT IRQ 0 - INPUT FOR DCC from cc1101 
	    EXT INT0 gives us an IRQ on both a high and a low change of the input stream */
    if(PIND & 0x04)                         // if it's a one, start of pulse
#endif
    {                                       // so, we need to
        usec = TCNT1;                       //   snag the current time in microseconds
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
                
                switch(byteCounter)
                {
                  case 3:
                    errorByte  = buffer[0];                 // VERY IMPORTANT!
                    errorByte ^= buffer[1];                 // All sorts of stuff flies around on the bus
                    
                    if (errorByte == buffer[2])
                    {
                        buffer[sizeof(DCC_MSG)-1] = byteCounter;        	// save length
                        buffer[sizeof(DCC_MSG)] = 0;

                        for (i=0;i<sizeof(DCC_MSG);i++)     // Move message to buffer for background task
                            dccbuff[i] = buffer[i];

                        transitionCountDCC = 0;                // Reset the transition count
                        setScheduledTask(TASK1);            // Schedule the background task
                    }
                  break;
                
                  case 4:
                    errorByte  = buffer[0];                 // XOR across all 
                    errorByte ^= buffer[1];
                    errorByte ^= buffer[2];                 // be VERY picky about what we accept
                    
                    if (errorByte == buffer[3])
                    {
                        buffer[sizeof(DCC_MSG)-1] = byteCounter;        	// save length
                        buffer[sizeof(DCC_MSG)] = 0;
                                                            // move out of operations buffer into background buffer
                        for (i=0;i<sizeof(DCC_MSG);i++)
                            dccbuff[i] = buffer[i];

                        transitionCountDCC = 0;            // Reset the transition count
                        setScheduledTask(TASK1);        // Schedule the background task
                    }                        
                  break;
                
                  case 5:
                    errorByte  = buffer[0];                 // Compute checksum (xor) across all
                    errorByte ^= buffer[1];
                    errorByte ^= buffer[2];
                    errorByte ^= buffer[3];

                    if (errorByte == buffer[4])             // if it matches, valid message came in
                    {
                        buffer[sizeof(DCC_MSG)-1] = byteCounter;        	// save length
                        buffer[sizeof(DCC_MSG)] = 0;

                        for (i=0;i<sizeof(DCC_MSG);i++)
                            dccbuff[i] = buffer[i];

                        transitionCountDCC = 0;                // Reset the transition count
                        setScheduledTask(TASK1);            // Schedule the background task
                    }
                  break;

                  case 6:
                    errorByte  = buffer[0];                 // Compute checksum (xor) across all
                    errorByte ^= buffer[1];
                    errorByte ^= buffer[2];
                    errorByte ^= buffer[3];
                    errorByte ^= buffer[4];

                    if (errorByte == buffer[5])             // if it matches, valid message came in
                    {
                        buffer[sizeof(DCC_MSG)-1] = byteCounter;        	// save length
                        buffer[sizeof(DCC_MSG)] = 0;

                        for (i=0;i<sizeof(DCC_MSG);i++)
                            dccbuff[i] = buffer[i];

                        transitionCountDCC = 0;                // Reset the transition count
                        setScheduledTask(TASK1);            // Schedule the background task
                    }
                  break;
                } // end of switch
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

