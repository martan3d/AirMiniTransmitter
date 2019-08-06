/*
 * spi.c
 *
 * Created: 12/2/2018 9:24:48 AM
 *  Author: martan
 */ 
#include <avr/io.h>
#include "spi.h"

uint8_t powerLevel=6; // The power level will be reset by reading EEPROM. Setting it here is possibly-important to prevent burn-out at higher levels

uint8_t initData[48] = {0x40, 0x2E, 0x2E, 0x0D, 0x07, 0xD3, 0x91, 0xFF,
                        0x04, 0x32, 0x00, 0x4B, 0x06, 0x00, 0x22, 0xB7,
                        0x55, 0x8A, 0x93, 0x00, 0x23, 0x3B, 0x50, 0x07,
                        0x30, 0x18, 0x16, 0x6C, 0x03, 0x40, 0x91, 0x87,
                        0x6B, 0xF8, 0x56, 0x10, 0xE9, 0x2A, 0x00, 0x1F,
                        0x40, 0x00, 0x59, 0x7F, 0x3F, 0x81, 0x35, 0x09};
						
						
	
uint8_t initTXData[48] = {0x40,   // address byte, start with reg 0, in burst mode
                          0x2E,   // IOCFG2
                          0x2E,   // IOCFG1 
                          0x0D,   // IOCFG0
                          0x07,   // FIFOTHR
                          0xD3,   // SYNC1
                          0x91,   // SYNC0
                          0xFF,   // PKTLEN
                          0x04,   // PKTCTRL1
                          0x32,   // PKTCTRL0
                          0x00,   // ADDR
                          0x4B,   // CHANNR
                          0x0c,   // FSCTRL1
                          0x00,   // FSCTRL0
                          0x22,   // FREQ2
                          0xB7,   // FREQ1
                          0x55,   // FREQ0
                          0x8C,   // MDMCFG4
                          0x22,   // MDMCFG3
                          0x93,   // MDMCFG2
                          0x23,   // MDMCFG1                          
                          0x3C,   // MDMCFG0
                          0x47,   // DEVIATN
                          0x07,   // MCSM2
                          0x30,   // MCSM1
                          0x18,   // MCSM0
                          0x16,   // FOCCFG
                          0x6C,   // BSCFG
                          0x03,   // AGCCTRL2
                          0x40,   // AGCCTRL1
                          0x91,   // AGCCTRL0
                          0x87,   // WOREVT1
                          0x6B,   // WOREVT0
                          0xF8,   // WORCTRL
                          0x56,   // FREND1    0101 0110
                          0x10,   // FREND0    0001 0000
                          0xE9,   // FSCAL3
                          0x2A,   // FSCAL2
                          0x00,   // FSCAL1
                          0x1F,   // FSCAL0
                          0x40,   // RCCTRL1
                          0x00,   // RCCTRL0
                          0x59,   // FSTEST
                          0x7F,   // PTEST
                          0x3F,   // AGCTEST
                          0x81,   // TEST2
                          0x35,   // TEST1
                          0x09    // TEST0
};	


// Channels designations are 0-16.  These are the corresponding values
// for the CC1101.
uint8_t channels[17] = {0x4B, 0x45, 0x33, 0x27, 0x1B, 0x15, 0x0F, 0x03, 0x5E,
                        0x58, 0x52, 0x3E, 0x39, 0x2C, 0x21, 0x89, 0x37};

// Transmitter power settings are designated 0-10.  These are the corresponding
// PATABLE entries to set these powers.

// See Table 4 of swrt151a.pdf (0xC0 is the highest level of output)
uint8_t powers[11] = {0x03, 0x15, 0x1C, 0x27, 0x66, 0x8E, 0x89, 0xCD, 0xC4,0xC1, 0xC0}; 

#define RX      0x34
#define TX      0x35
#define STOP    0x36
#define PATABLE 0x3E
#define CHAN    0x0A
#define SS      0x04
#define SNOP    0x3d

#define WRITE_BURST              0x40
#define READ_SINGLE              0x80
#define READ_BURST               0xC0


void initializeSPI()
{
    /* Set MOSI and SCK output, all others input */
    DDRB = 0x2f;
    PORTB |= SS;              // disable modem
    SPCR = 0x52;
}

uint8_t clockSPI(uint8_t data)
{
    SPDR = data;              // RX
    while(! (SPSR & 0x80) );  // wait for byte to clock out
    return (SPDR);
}


void writeReg(uint8_t reg, unsigned int data)
{
    PORTB &= ~SS;                // select modem (port low)

    clockSPI(reg);              // Channel Command
    clockSPI(data);

    PORTB |= SS;

}



void startModem(uint8_t channel, uint8_t mode)
{
    uint8_t i;
    uint8_t *md;
    uint8_t powerCode = 0x89;               //use 0x89 for rx mode
            powerCode = powers[powerLevel]; // Reset
    uint8_t channelCode = channels[channel];
    
	if (mode == RX) 
		md = initData;
	else
		md = initTXData;
	
    sendReceive(STOP);           // send stop command to modem

    PORTB &= ~SS;                // select modem (port low)
    for(i=0; i<48; i++) {
       clockSPI(md[i]);
	}
    PORTB |= SS;                 // disable modem

    PORTB &= ~SS;                // select modem (port low)
    clockSPI(PATABLE);           // Power Command
    clockSPI(powerCode);         // And hardcoded for RX
    PORTB |= SS;                 // disable modem

    PORTB &= ~SS;                // select modem (port low)
    clockSPI(CHAN);              // Channel Command
    clockSPI(channelCode);
    PORTB |= SS;                 // disable modem
 
    sendReceive(mode);           // RX mode
}


uint8_t sendReceive(uint8_t data)
{
    PORTB &= ~SS;             // select modem (port low)
    SPDR = data;              // RX
    while(! (SPSR & 0x80) );  // wait for byte to clock out
    PORTB |= SS;              // disable modem
    return (SPDR);
}

uint8_t readReg(uint8_t addr)
{
    uint8_t ret;
    
    PORTB &= ~SS;             // select modem (port low)
    SPDR = addr;              // RX
    while(! (SPSR & 0x80) );  // wait for byte to clock out
    SPDR = 0;                 // generic out, we only want read back
    while(! (SPSR & 0x80) );  // wait for byte to clock out
    ret = SPDR;               // readback
    PORTB |= SS;              // disable modem
    return (ret);
}

 
