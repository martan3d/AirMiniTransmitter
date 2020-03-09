/*
spi.c

Created: 12/2/2018 9:24:48 AM
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

/*
Important! 
The small SMD Anaren transciever boards operate at 27MHz!  These
boards (A110LR09C00GM with CC110L chips) won't work UNLESS
USE_WIRELESS_DCC_DATA is defined!  The related initRxData came from wireless-dcc.

The larger, board-level transciever boards operate at 26MHz!  They
will NOT work if USE_WIRELESS_DCC_DATA is defined!
*/

#include <avr/io.h>
#include "spi.h"

#ifdef TWENTY_SEVEN_MHZ
#define USE_WIRELESS_DCC_DATA
#else
#undef USE_WIRELESS_DCC_DATA
#endif

uint8_t powerLevel=6; // The power level will be reset by reading EEPROM. Setting it here is possibly-important to prevent burn-out at higher levels

#ifndef USE_WIRELESS_DCC_DATA
uint8_t initRxData[48] = {0x40, // address byte, start with reg 0, in burst mode
                          0x2E, // IOCFG2  // High impedance (3-state)
                          0x2E, // IOCFG1  // High impedance (3-state)
                          0x0D, // IOCFG0  // Serial Data Output. Asynchronous serial mode
                          0x07, // FIFOTHR
                          0xD3, // SYNC1
                          0x91, // SYNC0
                          0xFF, // PKTLEN
                          0x04, // PKTCTRL1 // Append payload with status bytes, no address check
                          0x32, // PKTCTRL0 // Asynchronous serial mode, infinite packet length
                          0x00, // ADDR
                          0x4B, // CHANNR
                          0x06, // FSCTRL1* (Reset value: 0x0F)
                          0x00, // FSCTRL0
#ifdef FCC_IC_APPROVED
                          0x22, // FREQ2
                          0xB7, // FREQ1
                          0x55, // FREQ0
#else
                          0x21, // FREQ2 Target fCarrier: 869.850Mhz
                          0x74, // FREQ1
                          0xAD, // FREQ0
#endif
                          0x8A, // MDMCFG4* // Changed as a test in conjunction w/ MDMCFG3
                          0x93, // MDMCFG3* // Changed as a test in conjunction w/ MDMCFG4
                          0x00, // MDMCFG2*
                          0x23, // MDMCFG1
                          0x3B, // MDMCFG0*
                          0x50, // DEVIATN*
                          0x07, // MCSM2
                          0x30, // MCSM1
                          0x18, // MCSM0
                          0x16, // FOCCFG
                          0x6C, // BSCFG
                          0x03, // AGCCTRL2
                          0x40, // AGCCTRL1
                          0x91, // AGCCTRL0
                          0x87, // WOREVT1
                          0x6B, // WOREVT0
                          0xF8, // WORCTRL
                          0x56, // FREND1    0101 0110
                          0x10, // FREND0    0001 0000
                          0xE9, // FSCAL3
                          0x2A, // FSCAL2
                          0x00, // FSCAL1
                          0x1F, // FSCAL0
                          0x40, // RCCTRL1
                          0x00, // RCCTRL0
                          0x59, // FSTEST
                          0x7F, // PTEST
                          0x3F, // AGCTEST
                          0x81, // TEST2
                          0x35, // TEST1
                          0x09};// TEST0
#else
#ifdef CE_APPROVED
uint8_t initRxData[48] = {0x40, // address byte, start with reg 0, in burst mode
                          0x2E, // IOCFG2  // High impedance (3-state)
                          0x2E, // IOCFG1  // High impedance (3-state)
                          0x0D, // IOCFG0  // Serial Data Output. Asynchronous serial mode
                          0x07, // FIFOTHR
                          0xD3, // SYNC1
                          0x91, // SYNC0
                          0xFF, // PKTLEN
                          0x04, // PKTCTRL1 // Append payload with status bytes, no address check
                          0x32, // PKTCTRL0 // Asynchronous serial mode, infinite packet length
                          0x00, // ADDR
                          0x4B, // CHANNR
                          0x06, // FSCTRL1* (Reset value: 0x0F)
                          0x00, // FSCTRL0
#ifdef FCC_IC_APPROVED
                          0x21, // FREQ2*
                          0x6E, // FREQ1*
                          0x2C, // FREQ0*
#else
                          0x20, // FREQ2*
                          0x37, // FREQ1*
                          0x77, // FREQ0* Exactly at 869.85MHz
#endif
                          0x8A, // MDMCFG4* // Changed as a test in conjunction w/ MDMCFG3
                          0x93, // MDMCFG3* // Changed as a test in conjunction w/ MDMCFG4
                          0x00, // MDMCFG2*
                          0x23, // MDMCFG1
                          0x3B, // MDMCFG0*
                          0x47, // DEVIATN*
                          0x07, // MCSM2
                          0x30, // MCSM1
                          0x18, // MCSM0
                          0x16, // FOCCFG
                          0x6C, // BSCFG
                          0x03, // AGCCTRL2
                          0x40, // AGCCTRL1
                          0x91, // AGCCTRL0
                          0x87, // WOREVT1
                          0x6B, // WOREVT0
                          0xF8, // WORCTRL
                          0x56, // FREND1    0101 0110
                          0x10, // FREND0    0001 0000
                          0xE9, // FSCAL3
                          0x2A, // FSCAL2
                          0x00, // FSCAL1
                          0x1F, // FSCAL0
                          0x40, // RCCTRL1
                          0x00, // RCCTRL0
                          0x59, // FSTEST
                          0x7F, // PTEST
                          0x3F, // AGCTEST
                          0x81, // TEST2
                          0x35, // TEST1
                          0x09};// TEST0
#else
uint8_t initRxData[48] = {0x40, // address byte, start with reg 0, in burst mode
                          0x2E, // IOCFG2
                          0x2E, // IOCFG1
                          0x0D, // IOCFG0  // Serial Data Output. Asynchronous serial mode
                          0x07, // FIFOTHR
                          0xD3, // SYNC1
                          0x91, // SYNC0
                          0xFF, // PKTLEN
                          0x04, // PKTCTRL1 // Append payload with status bytes, no address check
                          0x32, // PKTCTRL0 // Asynchronous serial mode, infinite packet length
                          0x00, // ADDR
                          0x4B, // CHANNR
                          0x06, // FSCTRL1* (Reset value: 0x0F)
                          0x00, // FSCTRL0
#ifdef FCC_IC_APPROVED
                          0x21, // FREQ2*
                          0x6E, // FREQ1*
                          0x2C, // FREQ0*
#else
                          0x20, // FREQ2*
                          0x37, // FREQ1*
                          0x77, // FREQ0* Exactly at 869.85MHz
#endif
                          0xBA, // MDMCFG4* 
                          0x84, // MDMCFG3*
                          0x00, // MDMCFG2*
                          0x23, // MDMCFG1
                          0x2F, // MDMCFG0*
                          0x47, // DEVIATN*
                          0x07, // MCSM2
                          0x30, // MCSM1
                          0x18, // MCSM0
                          0x16, // FOCCFG
                          0x6C, // BSCFG
                          0x03, // AGCCTRL2
                          0x40, // AGCCTRL1
                          0x91, // AGCCTRL0
                          0x87, // WOREVT1
                          0x6B, // WOREVT0
                          0xFB, // WORCTRL
                          0x56, // FREND1    0101 0110
                          0x10, // FREND0    0001 0000
                          0xE9, // FSCAL3
                          0x2A, // FSCAL2
                          0x00, // FSCAL1
                          0x1F, // FSCAL0
                          0x40, // RCCTRL1
                          0x00, // RCCTRL0
                          0x59, // FSTEST
                          0x7F, // PTEST
                          0x3F, // AGCTEST
                          0x81, // TEST2
                          0x35, // TEST1
                          0x09};// TEST0
#endif
#endif


#ifndef USE_WIRELESS_DCC_DATA
uint8_t initTxData[48] = {0x40,   // address byte, start with reg 0, in burst mode
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
                          0x0C,   // FSCTRL1*
                          0x00,   // FSCTRL0
#ifdef FCC_IC_APPROVED
                          0x22, // FREQ2
                          0xB7, // FREQ1
                          0x55, // FREQ0
#else
                          0x21, // FREQ2 Target fCarrier: 869.850Mhz
                          0x74, // FREQ1
                          0xAD, // FREQ0
#endif
                          0x8C,   // MDMCFG4*
                          0x22,   // MDMCFG3*
                          0x93,   // MDMCFG2*
                          0x23,   // MDMCFG1 
                          0x3C,   // MDMCFG0*
                          0x50,   // DEVIATN*
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
                          0x09};  // TEST0
#else
#ifdef CE_APPROVED
uint8_t initTxData[48] = {0x40,   // address byte, start with reg 0, in burst mode
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
                          0x0C,   // FSCTRL1*
                          0x00,   // FSCTRL0
#ifdef FCC_IC_APPROVED
                          0x21, // FREQ2*
                          0x6E, // FREQ1*
                          0x2C, // FREQ0*
#else
                          0x20, // FREQ2*
                          0x37, // FREQ1*
                          0x77, // FREQ0* Exactly at 869.85MHz
#endif
                          0x8C,   // MDMCFG4*
                          0x22,   // MDMCFG3*
                          0x93,   // MDMCFG2*
                          0x23,   // MDMCFG1 
                          0x3C,   // MDMCFG0*
                          0x47,   // DEVIATN*
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
                          0x09};  // TEST0
#else
uint8_t initTxData[48] = {0x40,    // address byte, start with reg 0, in burst mode
                          0x2E,    // IOCFG2
                          0x2E,    // IOCFG1 
                          0x0D,    // IOCFG0
                          0x07,    // FIFOTHR
                          0xD3,    // SYNC1
                          0x91,    // SYNC0
                          0xFF,    // PKTLEN
                          0x04,    // PKTCTRL1
                          0x32,    // PKTCTRL0
                          0x00,    // ADDR
                          0x4B,    // CHANNR
                          0x06,    // FSCTRL1*
                          0x00,    // FSCTRL0
#ifdef FCC_IC_APPROVED
                          0x21, // FREQ2*
                          0x6E, // FREQ1*
                          0x2C, // FREQ0*
#else
                          0x20, // FREQ2*
                          0x37, // FREQ1*
                          0x77, // FREQ0* Exactly at 869.85MHz
#endif
                          0xBA,    // MDMCFG4*
                          0x84,    // MDMCFG3*
                          0x00,    // MDMCFG2*
                          0x23,    // MDMCFG1 
                          0x2F,    // MDMCFG0*
                          0x47,    // DEVIATN
                          0x07,    // MCSM2
                          0x30,    // MCSM1
                          0x18,    // MCSM0
                          0x16,    // FOCCFG
                          0x6C,    // BSCFG
                          0x03,    // AGCCTRL2
                          0x40,    // AGCCTRL1
                          0x91,    // AGCCTRL0
                          0x87,    // WOREVT1
                          0x6B,    // WOREVT0
                          0xFB,    // WORCTRL
                          0x56,    // FREND1    0101 0110
                          0x10,    // FREND0    0001 0000
                          0xE9,    // FSCAL3
                          0x2A,    // FSCAL2
                          0x00,    // FSCAL1
                          0x1F,    // FSCAL0
                          0x40,    // RCCTRL1
                          0x00,    // RCCTRL0
                          0x59,    // FSTEST
                          0x7F,    // PTEST
                          0x3F,    // AGCTEST
                          0x81,    // TEST2
                          0x35,    // TEST1
                          0x09};   // TEST0
#endif
#endif


// Channels designations are 0-16.  These are the corresponding values
// for the CC1101.
// Note: corrected channel 15(0x89 -> 0x09 for a frequency of approximately 904.87MHz)
#ifdef FCC_IC_APPROVED
uint8_t channels[] = {0x4B, 0x45, 0x33, 0x27, 0x1B, 0x15, 0x0F, 0x03, 0x5E,
                      0x58, 0x52, 0x3E, 0x39, 0x2C, 0x21, 0x09, 0x37};
#else
uint8_t channels[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif

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

#define WRITE_BURST 0x40
#define READ_SINGLE 0x80
#define READ_BURST  0xC0


void initializeSPI()
{
                              //   |---------------------------------------------
                              //   | |---------------------------               |
                              //   | ||--------------           |               |
                              //   | ||             |           |               |
    DDRB = 0x2f;              // 001011 11 Set CSN (P10), MOSI (P11), and SCLK (P13) to outputs. Output Pins 8, 9 are not used.
                              //    |
                              //    MISO (P12) is input
    PORTB |= SS;              // 000001 00 disable modem on CSN (P10)
 // SPCR = 0x52;              // 01010010 Serial Port Control Register setting
    SPCR = 0x53;              // 01010011 Serial Port Control Register setting
                              // ||||||||
                              // ||||||SPR1, SPR0: Next slowest speed (10), Slowest speed (11)
                              // |||||CPHA: Rising edge sampling (0)
                              // ||||CPOL: Clock idle when low (0)
                              // |||MSTR: Arduino in master mode (1)
                              // ||DORD: MSB first (0)
                              // |SPE: Enable SPI (1)
                              // SPIE: Disable SPI Interrupt (0)
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
    if (channel > sizeof(channels)-1) channel=sizeof(channel)-1; // Error checking on channel
    uint8_t channelCode = channels[channel];
    
        if (mode == RX) 
           md = initRxData;
        else
           md = initTxData;

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
 
    sendReceive(mode);           // TX or RX mode
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

 
