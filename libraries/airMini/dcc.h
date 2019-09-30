/*
 * dcc.h
 *
 * Created: 10/14/2015 6:26:13 PM
 *  Author: martan
 */ 
 
#ifdef __cplusplus
extern "C" {
#endif

#include <avr/io.h>

#ifndef DCC_H_
#define DCC_H_

// Define here the input/output pins

#ifdef TRANSMIT
#define INPUT_PIN PD3
#define OUTPUT_PIN PD4
#else
#define INPUT_PIN PD2
#define OUTPUT_PIN PD3
#endif

#define SET_INPUTPIN DDRD &= ~(1<<INPUT_PIN)
#define SET_OUTPUTPIN DDRD |= (1<<OUTPUT_PIN)
#define OUTPUT_HIGH PORTD |=  (1<<OUTPUT_PIN)
#define OUTPUT_LOW  PORTD &= ~(1<<OUTPUT_PIN)

#define NOTHING 1
#define DCCMAXLEN 6 // This is the maximum # of bytes of a DCC packet (not 5!)
typedef struct
{
    uint8_t Data[DCCMAXLEN];
    uint8_t Size;
	
} DCC_MSG ;

void dccInit(void);
uint8_t * getDCC();
uint8_t decodeDCCPacket( DCC_MSG * dccptr);
uint16_t getTransitionCount();
void resetTransitionCount(uint16_t count);
void DCCuseModemData(uint8_t useModemData_in,uint8_t muxCVval_in);

#endif /* DCC_H_ */

#ifdef __cplusplus
}
#endif
