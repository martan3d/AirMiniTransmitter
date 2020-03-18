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
TWENTY_SEVEN_MHZ is defined!  The related initRxData came from wireless-dcc.

The larger, board-level transciever boards operate at 26MHz!
This requires #undef TWENTY_SEVEN_MHZ
*/

#include <avr/io.h>
#include "spi.h"

uint8_t na_operation = 1;

uint8_t powerLevel=6; // The power level will be reset by reading EEPROM. Setting it here is possibly-important to prevent burn-out at higher levels

// init[RT]xData settings.
//                         |    |    |    |    |    |    |    |    |    |    |    |    FSCTRL1 (Dep on fXOSC) IF Freq
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    FSCTRL0 (Dep on fXOSC) IF Freq
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    FREQ2 (Dep on desire Base Freq & fXOSC) Base Freq
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FREQ1 (Dep on desired Base Freq & fXOSC) Base Freq
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FREQ0 (Dep on desired Base Freq & fXOSC) Base Freq
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MDMCFG4 (Dep on fXOSC) CHANBW_E[7:6], CHANBW_M[5:4], DRATE_E[3:0]
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MDMCFG3 (Dep on fXOSC) DRATE_M[7:0]
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MDMCFG2 DEM_DCFILT_OFF[7:7], MOD_FORMAT[6:4], MANCHESTER_EN[3:3], SYNC_MODE[2:0]
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MDMCFG1 FEC_EN[7:7], NUM_PREAMBLE[6:4], NOT_USED[3:2], CHANSPC_E[1:0]
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MDMCFG0 (Dep on fXOSC) CHANSPC_M[7:0]
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    DEVIATN (Dep on fXOSC) NOT_USED[7:7], DEVIATION_E[6:4], NOT_USED[3:3], DEVIATION_M[2:0]
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    *    *    *    *    *    |    |    *    *    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
#define Rx_26MHz_NA        0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x22,0xB7,0x55,0x8A,0x93,0x00,0x23,0x3B,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Tx_26MHz_NA        0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x22,0xB7,0x55,0x8C,0x22,0x00,0x23,0x3B,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Rx_26MHz_EU        0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x21,0x74,0xAD,0x8A,0x93,0x00,0x23,0x3B,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Tx_26MHz_EU        0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x21,0x74,0xAD,0x8C,0x22,0x00,0x23,0x3B,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 

#define Rx_27MHz_NA        0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x21,0x6E,0x2C,0x8A,0x93,0x00,0x23,0x2F,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Tx_27MHz_NA        0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x21,0x6E,0x2C,0x8C,0x22,0x00,0x23,0x2F,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Rx_27MHz_EU        0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x20,0x37,0x77,0x8A,0x93,0x00,0x23,0x2F,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Tx_27MHz_EU        0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x20,0x37,0x77,0x8C,0x22,0x00,0x23,0x2F,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 

// Experimental only
// Change to GFSK from 2-FSK. Result: None to perhaps slightly negative for CVP receivers.
//                                                                                                                        *          
#define Tx_27MHz_NA_E      0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x21,0x6E,0x2C,0x8C,0x22,0x10,0x23,0x2F,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 

// Works
#if defined(TWENTY_SEVEN_MHZ)
#pragma message "Info: using Rx_27MHz_NA. Works with CVP transmitters"
uint8_t initRxData_na[48] = {
Rx_27MHz_NA
};
#pragma message "Info: using Rx_27MHz_EU. Works with Tam Valley Depot EU DRS1 transmitters"
uint8_t initRxData_eu[48] = {
Rx_27MHz_EU
};
#pragma message "Info: using Tx_27MHz_NA. Works with CVP and Tam Valley Depot receivers"
uint8_t initTxData_na[48] = {
Tx_27MHz_NA
};
#pragma message "Info: using Tx_27MHz_EU. Works with Tam Valley Depot EU DRS1 receivers"
uint8_t initTxData_eu[48] = {
Tx_27MHz_EU
};
#endif

// Works
#if !defined(TWENTY_SEVEN_MHZ)
#pragma message "Info: using Rx_26MHz_NA. Works with CVP transmitters"
uint8_t initRxData_na[48] = {
Rx_26MHz_NA
};
#pragma message "Info: using Rx_26MHz_EU. Works with Tam Valley Depot EU DRS1 transmitters"
uint8_t initRxData_eu[48] = {
Rx_26MHz_EU
};
#pragma message "Info: using Tx_26MHz_NA. Works with CVP and Tam Valley Depot recievers"
uint8_t initTxData_na[48] = {
Tx_26MHz_NA
};
#pragma message "Info: using Tx_26MHz_EU. Works with Tam Valley Depot EU DRS1 receivers"
uint8_t initTxData_eu[48] = {
Tx_26MHz_EU
};
#endif


// Channels designations are 0-16.  These are the corresponding values
// for the CC1101.
// Note: corrected channel 15(0x89 -> 0x09 for a frequency of approximately 904.87MHz)
uint8_t channels_na[17] = {0x4B, 0x45, 0x33, 0x27, 0x1B, 0x15, 0x0F, 0x03, 0x5E,
                           0x58, 0x52, 0x3E, 0x39, 0x2C, 0x21, 0x09, 0x37};
uint8_t channels_eu[1]  = {0x00};
uint8_t channels_na_max = sizeof(channels_na)-1;
uint8_t channels_max = sizeof(channels_na)+sizeof(channels_eu)-1;

// Transmitter power settings are designated 0-10.  These are the corresponding
// PATABLE entries to set these powers.

// See Table 4 of swra151a.pdf (0xC0 is the highest level of output @9.5dBM)
// Removed 0x66 entry (-4.9dBM) per the note in this document
//dBm                    -29.8 -22.8 -16.1 -9.7  -4.7  -0.6  2.2   5.0   7.9   9.0   9.4
uint8_t powers_na[11] = {0x03, 0x15, 0x1C, 0x27, 0x56, 0x8E, 0x89, 0xCD, 0xC4, 0xC1, 0xC0}; 
// See Table 3 of swra151a.pdf (0xC0 is the highest level of output @9.2dBm)
// dbm                   -30.2 -23.0 -16.4 -9.8  -4.8  -0.5  2.1   5.0   7.8   8.9   9.2
uint8_t powers_eu[11] = {0x03, 0x15, 0x1C, 0x27, 0x57, 0x8E, 0x8A, 0x81, 0xC8, 0xC5, 0xC4}; 

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
    uint8_t channelCode;
    uint8_t powerCode;
    uint8_t channel_l = channel % (channels_max+1); // Error checking on channel
    if (channel_l <= channels_na_max){
       na_operation = 1;
       channelCode = channels_na[channel_l];
       powerCode = powers_na[powerLevel]; // Reset
    }
    else {
       channel_l -= (channels_na_max+1);
       na_operation = 0;
       channelCode = channels_eu[channel_l];
       powerCode = powers_eu[powerLevel]; // Reset
    }
    

    if (mode == RX) {
       if (na_operation) md = initRxData_na;
       else md = initRxData_eu;
    }
    else {
       if (na_operation) md = initTxData_na;
       else md = initTxData_eu;
    }

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
    SPDR = data;              // RX|TX
    while(! (SPSR & 0x80) );  // wait for byte to clock out
    PORTB |= SS;              // disable modem
    return (SPDR);
}

uint8_t readReg(uint8_t addr)
{
    uint8_t ret;
    
    PORTB &= ~SS;             // select modem (port low)
    SPDR = addr;              // RX|TX
    while(! (SPSR & 0x80) );  // wait for byte to clock out
    SPDR = 0;                 // generic out, we only want read back
    while(! (SPSR & 0x80) );  // wait for byte to clock out
    ret = SPDR;               // readback
    PORTB |= SS;              // disable modem
    return (ret);
}

 
