/*
spi.c

Created: 12/2/2018 9:24:48 AM
Copyright (c) 2018-2019, Martin Sant and Darrell Lamm
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
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

uint8_t region = 0;

uint8_t powerLevel=6; // The power level will be reset by reading EEPROM. Setting it here is possibly-important to prevent burn-out at higher levels

// init[RT]xData settings.
//                         Burst mode
//                         |    IOCFG0(0x00) High Impedance (3-state)
//                         |    |    IOCFG1(0x01) High Impedance (3-state)
//                         |    |    |    IOCFG2(0x02) 0x0D Serial Data Output. Used for asynchronous serial mode
//                         |    |    |    |    FIFOTHR (0x3) Reserved for future use [7:4], set to b000; TX/RX FIFO threshold[3:0], b111=7: Tx/Rx thresholds = 33/32
//                         |    |    |    |    |    SYNC1 (0x4) Sync word high, byte
//                         |    |    |    |    |    |    SYNC0 (0x5) Sync word,low byte
//                         |    |    |    |    |    |    |    PKTLEN (0x06)
//                         |    |    |    |    |    |    |    |    PKTCTRL1 (0x07) 2:2 appended with status bytes; 1:0 Addressing (=b00 for here no address check)
//                         |    |    |    |    |    |    |    |    |    PKCTCTRL0 (0x08) Special setting for asynchronous data: [5:4] =b11 for asynchronous data; 1:0 =b10 for infinite length data
//                         |    |    |    |    |    |    |    |    |    |    ADDR (0x09) Address =(b00000000) we have no address
//                         |    |    |    |    |    |    |    |    |    |    |    CHANNR (0x0A) Channel #, which will be changed!
//                         |    |    |    |    |    |    |    |    |    |    |    |    FSCTRL1 (0x0B) (Dep on fXOSC) IF Freq
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    FSCTRL0 (0x0C) (Dep on fXOSC) IF Freq
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    FREQ2 (0xD) (Dep on desired Base Freq & fXOSC) Base Freq
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FREQ1 (0xE) (Dep on desired Base Freq & fXOSC) Base Freq
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FREQ0 (0xF) (Dep on desired Base Freq & fXOSC) Base Freq
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MDMCFG4 (0x10) (Dep on fXOSC) CHANBW_E[7:6], CHANBW_M[5:4], DRATE_E[3:0]
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MDMCFG3 (0x11)(Dep on fXOSC) DRATE_M[7:0]
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MDMCFG2 (0x12) DEM_DCFILT_OFF[7:7], MOD_FORMAT[6:4], MANCHESTER_EN[3:3], SYNC_MODE[2:0] (MOD_FORMAT: b000 -> 2-FSK; b001 -> GSFK)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MDMCFG1 (0x13) FEC_EN[7:7], NUM_PREAMBLE[6:4], NOT_USED[3:2], CHANSPC_E[1:0]
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MDMCFG0 (0x14) (Dep on fXOSC) CHANSPC_M[7:0]
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    DEVIATN (0x15) (Dep on fXOSC) NOT_USED[7:7], DEVIATION_E[6:4], NOT_USED[3:3], DEVIATION_M[2:0]
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MCSM2 (0x16) Main RC State Machine Conf (default vals)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MCSM1 (0x17) Main RC State Machine Conf (default vals)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    MCSM0 (0x18) Main RC State Machine Conf. ?
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FOCCFG (0x19) Freq Offset Comp. Conf. ?
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    BSCFG (0x1A) Bit Sync Conf. ?
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    AGCCTRL2 (0x1B) AGC Ctrl ?
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    AGCCTRL1 (0x1C) AGC Ctrl ?
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    AGCCTRL0 (0x1D) AGC Ctrl ?
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    WOREVT1 (0x1E) ? Skippped in RF Studio!
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    WOREVT0 (0x1F) ? Skippped in RF Studio!
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    WORCTRL(0x20) From RF Studio
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FREND1 (0x21) FE RX Config
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FREND0 (0x22) FE TX Config
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FSCAL3 (0x23)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FSCAL2 (0x24)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FSCAL1 (0x25)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FSCAL0 (0x26)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    RCCTRL1 (0x27) ? Skippped in RF Studio
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    RCCTRL0 (0x28) ? Skippped in RF Studio
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    FSTEST (0x29)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    PTEST (0x2A)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    AGCTEST (0x2B)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    TEST2 (0x2C)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    TEST1 (0x2D)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    TEST0 (0x2E)
//                         |    |    |    |    |    |    |    |    |    |    |    |    |    |    *    *    *    *    *    |    |    *    *    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
#if defined(NAEU_900MHz)
//{
//                                                                                *                   *    *    
//                                                                                                    916.48MHz (Bytes 0x0A, 0x0E, 0x0F for compatibility w/ Tam Valley Depot Tx)
//                                                                                                                                       0x60 for (100kHz deviation)
/*
#define Rx_26MHz_NA_916    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x23,0x3F,0xCE,0x8A,0xA2,0x00,0x23,0x3B,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Tx_26MHz_NA_916    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x23,0x3F,0xCE,0x8C,0xA2,0x00,0x23,0x3B,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
*/

//{
#define Rx_26MHz_NA_915    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x22,0xB7,0x55,0x8A,0x93,0x00,0x23,0x3B,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Tx_26MHz_NA_915    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x22,0xB7,0x55,0x8C,0x22,0x00,0x23,0x3B,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
//                                                                                *
#define Rx_26MHz_EU_869    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x21,0x74,0xAD,0x8A,0x93,0x00,0x23,0x3B,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Tx_26MHz_EU_869    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x21,0x74,0xAD,0x8C,0x22,0x00,0x23,0x3B,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
//}
#endif

#if defined(EU_434MHz)
//{
// F0=433.20MHz                                                                   *    *    *    *    *    *    *    *    *    *    *    *                             *
#define Rx_26MHz_EU_434    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x04,0x06,0x00,0x10,0xA9,0x5A,0xCA,0x93,0x00,0x22,0xF8,0x50,0x07,0x30,0x18,0x16,0x6C,0x43,0x40,0x91,0x87,0x6B,0xFB,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
/* F0=434MHz
#define Rx_26MHz_EU_434    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x10,0xB1,0x3B,0xCA,0x93,0x00,0x22,0xF6,0x50,0x07,0x30,0x18,0x16,0x6C,0x43,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
*/

// F0=433.20MHz                                                                   *    *    *    *    *    *    *    *    *    *    *    *                             *
#define Tx_26MHz_EU_434    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x04,0x06,0x00,0x10,0xA9,0x5A,0xCA,0x93,0x00,0x22,0xF8,0x50,0x07,0x30,0x18,0x16,0x6C,0x43,0x40,0x91,0x87,0x6B,0xFB,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
/* F0=434MHz
#define Tx_26MHz_EU_434    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x10,0xB1,0x3B,0xCA,0x93,0x00,0x22,0xF6,0x50,0x07,0x30,0x18,0x16,0x6C,0x43,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
*/
//}
#endif

#if defined(NAEU_2p4GHz)
//{
#if defined(ALTERNATIVE2P4)
#pragma message "Based on Table 3 of A2400R24x - User's Manual: 2433MHz Base Frequency, 26MHz Xtal Frequency, 2-FSK 0 Channel Number, 100kBaud, 140kHz Deviation, 310.242kHz Channel spacing, 650kHz RX Filter BW"
//                                             *                                  *    *    *    *    *    *    *    *    *    *    *    *                             *                                       *    *    *    *    *                        *    *    *
#define Rx_26MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x47,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5D,0x93,0xB1,0x1B,0xF8,0x00,0x23,0x87,0x63,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B
#define Tx_26MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x47,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5D,0x93,0xB1,0x1B,0xF8,0x00,0x23,0x87,0x63,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B
/*
// #pragma message "Based on Table 3 of A2400R24x - User's Manual: 2433MHz Base Frequency, 26MHz Xtal Frequency, 2-FSK 0 Channel Number, 38kBaud, 140kHz Deviation, 310.242kHz Channel spacing, 650kHz RX Filter BW"
#define Rx_26MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5D,0x93,0xB1,0x1A,0x7F,0x00,0x23,0x87,0x63,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B
#define Tx_26MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5D,0x93,0xB1,0x1A,0x7F,0x00,0x23,0x87,0x63,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B
*/
#else
#pragma message "2433MHz Base Frequency, 26MHz Xtal Frequency, 2-FSK, 0 Channel Number, 39.9704kBaud, 50.781250kHz Deviation, 199.554443kHZ Channel Spacing 101.562500kHz RX Filter BW"
#define Rx_26MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5D,0x93,0xB1,0xCA,0x93,0x00,0x22,0xF8,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B
#define Tx_26MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5D,0x93,0xB1,0xCA,0x93,0x00,0x22,0xF8,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B
#endif
//}
#endif

#if defined(NAEU_900MHz)
//{
//                                                                                *                   *    *    
//                                                                                                    916.48MHz (Bytes 0x0A, 0x0E, 0x0F for compatibility w/ Tam Valley Depot Tx)
//                                                                                                                                       0x57 for (100kHz deviation)
/*
#define Rx_27MHz_NA_916    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x21,0xF1,0x97,0x8A,0x93,0x00,0x23,0x2F,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Tx_27MHz_NA_916    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x21,0xF1,0x97,0x8C,0x93,0x00,0x23,0x2F,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
*/

#define Rx_27MHz_NA_915    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x21,0x6E,0x2C,0x8A,0x93,0x00,0x23,0x2F,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Tx_27MHz_NA_915    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x4B,0x06,0x00,0x21,0x6E,0x2C,0x8C,0x22,0x00,0x23,0x2F,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 

//                                                                                *
#define Rx_27MHz_EU_869    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x20,0x37,0x77,0x8A,0x93,0x00,0x23,0x2F,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
#define Tx_27MHz_EU_869    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x20,0x37,0x77,0x8C,0x22,0x00,0x23,0x2F,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x40,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
//}
#endif

#if defined(EU_434MHz)
//{
// F0=433.20MHz                                                                   *    *    *    *    *    *    *    *    *    *    *    *                             *                        *
#define Rx_27MHz_EU_434    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x04,0x06,0x00,0x10,0x0B,0x60,0xCA,0x84,0x00,0x22,0xE5,0x47,0x07,0x30,0x18,0x16,0x6C,0x43,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09
/* Original at F0=434MHz
#define Rx_27MHz_EU_434    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x10,0x12,0xF6,0xCA,0x84,0x00,0x22,0xE4,0x47,0x07,0x30,0x18,0x16,0x6C,0x43,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
*/
// F0=433.20MHz                                                                   *    *    *    *    *    *    *    *    *    *    *    *                             *                        *
#define Tx_27MHz_EU_434    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x04,0x06,0x00,0x10,0x0B,0x60,0xCA,0x84,0x00,0x22,0xE5,0x47,0x07,0x30,0x18,0x16,0x6C,0x43,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09
/* Original at F0=434MHz
#define Tx_27MHz_EU_434    0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x10,0x12,0xF6,0xCA,0x84,0x00,0x22,0xE4,0x47,0x07,0x30,0x18,0x16,0x6C,0x43,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x7F,0x3F,0x81,0x35,0x09 
*/
//}
#endif

#if defined(NAEU_2p4GHz)
//{
// Based on Table 3 of A2400R24x - User's Manual: 38kBaud, 140kHz Deviation, 310.242kHz Channel spacing, 650kHz RX Filter BW
//                                                                                *    *    *    *    *    *    *    *    *    *    *    *                             *                                       *    *    *    *    *                        *    *    *

#define Rx_27MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5A,0x1C,0x71,0xCA,0x84,0x00,0x22,0xE5,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B
//                                                                                *    *    *    *    *    *    *    *    *    *    *    *                             *                                       *    *    *    *    *                        *    *    *
#define Tx_27MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5A,0x1C,0x71,0xCA,0x84,0x00,0x22,0xE5,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B
//
/*
#define Rx_26MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5D,0x93,0xB1,0xCA,0x93,0x00,0x22,0xF7,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B 
#define Tx_26MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5D,0x93,0xB1,0xCA,0x93,0x00,0x22,0xF7,0x50,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B 
#define Rx_27MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5A,0x1C,0x71,0xCA,0x84,0x00,0x22,0xE4,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B 
#define Tx_27MHz_NAEU_2p4  0x40,0x2E,0x2E,0x0D,0x07,0xD3,0x91,0xFF,0x04,0x32,0x00,0x00,0x06,0x00,0x5A,0x1C,0x71,0xCA,0x84,0x00,0x22,0xE4,0x47,0x07,0x30,0x18,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x88,0x31,0x0B 
*/
// Works
//}
#endif
#if defined(TWENTY_SEVEN_MHZ)
//{

#if defined(NAEU_900MHz)
//{
#pragma message "Info: using Rx_27MHz_NA_915. Works with CVP transmitters"
#pragma message "Info: using Rx_27MHz_EU_869. Works with Tam Valley Depot EU DRS1 transmitters"
uint8_t initRxData[2][48] = {
{Rx_27MHz_NA_915},
{Rx_27MHz_EU_869}
};
#pragma message "Info: using Tx_27MHz_NA_915. Works with CVP and Tam Valley Depot receivers"
#pragma message "Info: using Tx_27MHz_EU_869. Works with Tam Valley Depot EU DRS1 receivers"
uint8_t initTxData[2][48] = {
{Tx_27MHz_NA_915},
{Tx_27MHz_EU_869}
};
// NAEU_900MHz
//}
#else
//{

#if defined(EU_434MHz)
//{
#pragma message "Info: using Rx_27MHz_EU_434. For repeater receivers only"
uint8_t initRxData[1][48] = {
{Rx_27MHz_EU_434}
};
#pragma message "Info: using Tx_27MHz_EU_434. For repeater transmitters only"
uint8_t initTxData[1][48] = {
{Tx_27MHz_EU_434}
};
// EU_434MHz
//}
#else
//{

#if defined(NAEU_2p4GHz)
//{
#pragma message "Info: using Rx_27MHz_NAEU_2p4. For repeater receivers only"
uint8_t initRxData[1][48] = {
{Rx_27MHz_NAEU_2p4}
};
#pragma message "Info: using Tx_27MHz_NAEU_2p4. For repeater transmitters only"
uint8_t initTxData[1][48] = {
{Tx_27MHz_NAEU_2p4}
};
// NAEU_2p4GHz
//}
#endif

// EU_434MHz
//}
#endif

// NAEU_900MHz
//}
#endif

// TWENTY_SEVEN_MHZ
//}
#else
//{

// Works
#if defined(TWENTY_SIX_MHZ)
//{

#if defined(NAEU_900MHz)
//{
#pragma message "Info: using Rx_26MHz_NA_915. Works with CVP transmitters"
#pragma message "Info: using Rx_26MHz_EU_869. Works with Tam Valley Depot EU DRS1 transmitters"
uint8_t initRxData[2][48] = {
{Rx_26MHz_NA_915},
{Rx_26MHz_EU_869}
};
#pragma message "Info: using Tx_26MHz_NA_915. Works with CVP and Tam Valley Depot receivers"
#pragma message "Info: using Tx_26MHz_EU_869. Works with Tam Valley Depot EU DRS1 receivers"
uint8_t initTxData[2][48] = {
{Tx_26MHz_NA_915},
{Tx_26MHz_EU_869}
};
// NAEU_900MHz
//}
#else
//{

#if defined(EU_434MHz)
//{
#pragma message "Info: using Rx_26MHz_EU_434. For repeater receivers only"
uint8_t initRxData[1][48] = {
{Rx_26MHz_EU_434}
};
#pragma message "Info: using Tx_26MHz_EU_434. For repeater transmitters only"
uint8_t initTxData[1][48] = {
{Tx_26MHz_EU_434}
};
// EU_434MHz
//}
#else
//{

#if defined(NAEU_2p4GHz)
//{
#pragma message "Info: using Rx_26MHz_NAEU_2p4. For repeater receivers only"
uint8_t initRxData[1][48] = {
{Rx_26MHz_NAEU_2p4}
};
#pragma message "Info: using Tx_26MHz_NAEU_2p4. For repeater transmitters only"
uint8_t initTxData[1][48] = {
{Tx_26MHz_NAEU_2p4}
};
// NAEU_2p4GHz
//}
#endif

// EU_434MHz
//}
#endif

// NAEU_900MHz
//}
#endif
// TWENTY_SIX_MHZ
//}
#else
//{
#error "Undefined crystal frequency"
// TWENTY_SIX_MHZ
//}
#endif

// TWENTY_SEVEN_MHZ
//}
#endif


#if defined(NAEU_900MHz)
//{
// Channels designations are 0-18.  These are the corresponding values
// for the CC1101.
// Note: corrected channel 15(0x89 -> 0x09 for a frequency of approximately 904.87MHz)
#pragma message "Info: using NA 915MHz and EU 869MHz channels (0-18)"
const uint8_t channels[19] = {0x4B, 0x45, 0x33, 0x27, 0x1B, 0x15, 0x0F, 0x03, 0x5E, // NA (0-8)
                              0x58, 0x52, 0x3E, 0x39, 0x2C, 0x21, 0x09, 0x37, 0x00, // NA (9-17)
                              0x00};                                                // EU (18)
uint8_t channels_na_max = 17;
// NAEU_900MHz
//}
#else
//{

#if defined(EU_434MHz)
//{
#pragma message "Info: using EU 434MHz channel (0-7)"
const uint8_t channels[8] = {0x04,0x00,0x05,0x01,0x06,0x02,0x07,0x03}; // EU (0-7) -4,+5,-4,+5,-4,+5,-4. Largest avg step sep w/ smallest rms
// EU_434MHz
//}
#else
//{

#if defined(NAEU_2p4GHz)
//{
#pragma message "Info: using Worldwide 2.4GHz channels (0-7)"
const uint8_t channels[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}; // W (0-7)
// NAEU_2p4GHz
//}
#endif

// EU_434MHz
//}
#endif

// NAEU_900MHz
//}
#endif
uint8_t channels_max = sizeof(channels) - 1;

// Transmitter power settings are designated 0-10.  These are the corresponding
// PATABLE entries to set these powers.

#if defined(NAEU_900MHz)
//{
#pragma message "Info: using NA 915MHz power table (-29.8dBm to 9.4dBm)"
#pragma message "Info: using EU 869MHz power table (-30.2dBm to 9.2dBm)"
// See Table 4 of swra151a.pdf (0xC0 is the highest level of output @9.5dBm)
// Removed 0x66 entry (-4.9dBm) per the note in this document
// dBm                          -29.8 -22.8 -16.1 -9.7  -4.7  -0.6  +2.2  +5.0  +7.9  +9.0  +9.4
const uint8_t powers[2][11] = {{0x03, 0x15, 0x1C, 0x27, 0x56, 0x8E, 0x89, 0xCD, 0xC4, 0xC1, 0xC0},
// See Table 3 of swra151a.pdf (0xC0 is the highest level of output @9.2dBm)
// dBm                          -30.2 -23.0 -16.4 -9.8  -4.8  -0.5  +2.1  +5.0  +7.8  +8.9  +9.2
                               {0x03, 0x15, 0x1C, 0x27, 0x57, 0x8E, 0x8A, 0x81, 0xC8, 0xC5, 0xC4}}; 
// NAEU_900MHz
//}
#else
//{

#if defined(EU_434MHz)
//{
#pragma message "Info: using EU 434MHz power table (-30dBm to +10.0dBm)"
//                              -30.0 -20.0 -15.0 -10.0 0.0   +5.0  +7.0  +10.0 +10.0 +10.0 +10.0
const uint8_t powers[1][11] = {{0x12, 0x0E, 0x1D, 0x34, 0x60, 0xB4, 0xC8, 0xC0, 0xC0, 0xC0, 0xC0}};
// EU_434MHz
//}
#else
//{

#if defined(NAEU_2p4GHz)
//{
// Based on Table 4 of A2400R24x - User's Manual
#pragma message "Info: using Worldwide 2.4GHz power table (-16dBm to 1.0dBm)"
// dBm                          -16.0 -14.0 -12.0 -10.0 -8.0  -6.0  -4.0  -2.0  0.0   +1.0  +1.0
const uint8_t powers[1][11] = {{0x55, 0x8D, 0xC6, 0x97, 0x6E, 0x7F, 0xA9, 0xBB, 0xFE, 0xFF, 0xFF}};
// NAEU_2p4GHz
//}
#endif

// EU_434MHz
//}
#endif

// NAEU_900MHz
//}
#endif

// Set DEVIATN register
#define DEVIATN 0x15
// Set CHAN register
#define CHAN    0x0A
// Set FREQ2 register
#define FREQ2   0x0D
// Set FREQ1 register
#define FREQ1   0x0E
// Set FREQ0 register
#define FREQ0   0x0F
// Reset command strobe
#define SRES    0x30
// Receive command strobe
#define RX      0x34
// Transmit command strobe
#define TX      0x35
// Idle command strobe
#define SIDLE   0x36
// Set power register
#define PATABLE 0x3E
// NOOP command strobe
#define SNOP    0x3D

#define WRITE_BURST 0x40
#define READ_SINGLE 0x80
#define READ_BURST  0xC0

// For 27MHz Tam Valley Rx compatibility
#if defined(TWENTY_SEVEN_MHZ)
#define FREQ2VAL 0x21
#define FREQ1VAL 0xF1
#define FREQ0VAL 0x97
#define CHANVAL  0x00
#define DEVIATNVAL 0x3F
#else
#define FREQ2VAL 0x23
#define FREQ1VAL 0x3F
#define FREQ0VAL 0xCE
#define CHANVAL  0x00
#define DEVIATNVAL 0x40
#endif

uint8_t deviatnval=DEVIATNVAL;
uint8_t deviatn_changed=0;

#define SCK_PIN  13
#define MISO_PIN 12
#define MOSI_PIN 11
#define SS_PIN   10
#define PIN_9     9
#define PIN_8     8

#define SPIFINTMASK (1 << SPIF) // SPIF interrupt mask

volatile uint8_t *pin8IntPort;  // use port and bitmask to write output in ISR
uint8_t pin8IntMask;            // digitalWrite is too slow on AVR

volatile uint8_t *pin9IntPort;  // use port and bitmask to write output in ISR
uint8_t pin9IntMask;            // digitalWrite is too slow on AVR

volatile uint8_t *ssIntPort;    // use port and bitmask to write output in ISR
uint8_t ssIntMask;              // digitalWrite is too slow on AVR
uint8_t ssIntMask_;             // digitalWrite is too slow on AVR

volatile uint8_t *misoIntPort;  // use port and bitmask to read input in ISR
uint8_t misoIntMask;            // digitalRead is too slow on AVR

volatile uint8_t *mosiIntPort;  // use port and bitmask to write output in ISR
uint8_t mosiIntMask;            // digitalWrite is too slow on AVR

volatile uint8_t *sckIntPort;   // use port and bitmask to write output in ISR
uint8_t sckIntMask;             // digitalWrite is too slow on AVR
uint8_t sckIntMask_;            // digitalWrite is too slow on AVR

void beginSPI() {
    *ssIntPort &= ssIntMask_;            // select modem (port low)
    while( *misoIntPort & misoIntMask ); // WAIT while MISO pin is HIGH
}

void endSPI() {
    *ssIntPort |= ssIntMask;             // unselect modem (port high)
    *sckIntPort &= sckIntMask_;          // set clock low (port low)
}

void resetModem() {
    *ssIntPort &= ssIntMask_;            // set ss low
    delay(1);
    *ssIntPort |= ssIntMask;             // set ss high
    delay(1);
    *ssIntPort &= ssIntMask_;            // set ss low
    while( *misoIntPort & misoIntMask ); // WAIT while MISO pin is HIGH
    strobeSPI(SRES);                     // send reset command to modem
    while( *misoIntPort & misoIntMask ); // WAIT while MISO pin is HIGH
    *ssIntPort |=  ssIntMask;            // set ss high
}

void initializeSPI()
{
    // PIN8 (P8) PORT# and BitMasks (output) - Unused
    pin8IntPort  = portOutputRegister( digitalPinToPort(PIN_8) );    // PORTB = *ssIntPort
    pin8IntMask  = digitalPinToBitMask(PIN_8); // High

    // PIN9 (P9) PORT# and BitMasks (output) - Unused
    pin9IntPort  = portOutputRegister( digitalPinToPort(PIN_9) );    // PORTB = *ssIntPort
    pin9IntMask  = digitalPinToBitMask(PIN_9); // High

    // CSN (P10) PORT# and BitMasks (output)
    ssIntPort  = portOutputRegister( digitalPinToPort(SS_PIN) );    // PORTB = *ssIntPort
    ssIntMask  = digitalPinToBitMask(SS_PIN); // High
    ssIntMask_ = ~ssIntMask; // Low. Saves time, more memory
 
    // MOSI (P11) PORT# and BitMask (output)
    mosiIntPort = portOutputRegister( digitalPinToPort(MOSI_PIN) ); // PORTB = *mosiIntPort
    mosiIntMask = digitalPinToBitMask(MOSI_PIN); // High

    // MISO (P12) PORT# and BitMask (input)
    misoIntPort = portInputRegister( digitalPinToPort(MISO_PIN) );  // PINB = *misoIntPort
    misoIntMask = digitalPinToBitMask(MISO_PIN); // High

    // SCK (P13) PORT# and BitMasks (output)
    sckIntPort  = portOutputRegister( digitalPinToPort(SCK_PIN) );  // PORTB = *sckIntPort
    sckIntMask  = digitalPinToBitMask(SCK_PIN); // High
    sckIntMask_ = ~sckIntMask; // Low. Saves time, more memory
 
                              //   |-----------------------------------------------
                              //   | |-----------------------------               |
                              //   | ||----------------           |               |
                              //   | ||               |           |               |
    DDRB = 0x2f;              // 00101111    Set CSN (P10), MOSI (P11), and SCLK (P13) to outputs. Output Pins 8, 9 are not used.
                              //    |
                              //    MISO (P12) is input

    *sckIntPort |= sckIntMask; // set clk high (port high)

    delay(10);

    endSPI();

#if ! defined(SPCRDEFAULT)
    SPCR = 0x52;              // 01010010 Serial Port Control Register setting
//  SPCR = 0x53;              // 01010011 Serial Port Control Register setting
                              // ||||||||
                              // ||||||SPR1, SPR0: Next slowest speed (10), Slowest speed (11)
                              // |||||CPHA: Rising edge sampling (0)
                              // ||||CPOL: Clock idle when low (0)
                              // |||MSTR: Arduino in master mode (1)
                              // ||DORD: MSB first (0)
                              // |SPE: Enable SPI (1)
                              // SPIE: Disable SPI Interrupt (0)
#else
    SPCR = SPCRDEFAULT;
#endif

    SPSR;               // Clear out any junk
    SPDR;               // Clear out any junk
}

uint8_t clockSPI(uint8_t data)
{
    SPDR = data;              // Send
    while (!(SPSR & SPIFINTMASK));  // wait for byte to clock out
    return (SPDR);
}


void writeSPI(uint8_t reg, uint8_t data)
{
    beginSPI();

    clockSPI(reg);              // recipent address
    clockSPI(data);

    endSPI();

}


void startModem(uint8_t channel, uint8_t mode)
{
    uint8_t i;
    uint8_t *md;
    uint8_t channelCode;
    uint8_t powerCode;
    uint8_t channel_l = channel % (channels_max+1); // Error checking on channel
    // Select the region
#if defined(NAEU_900MHz)
    if (channel_l <= channels_na_max)
       region = 0;
    else 
       region = 1;
#endif

    channelCode = channels[channel_l];
    powerCode = powers[region][powerLevel];

    if (mode == RX) {
       md = initRxData[region];
    }
    else {
       md = initTxData[region];
    }

    ////////////////
    strobeSPI(SIDLE);                // send stop command to modem (old way)
    
/*
    /////////////////////
    // New Reset sequence
    // This sequence was copied from the ELECHOUSE_CC1101_SRC_DRV.cpp file. It works for the CC2500 as well.
    // It is a combination of the Init() and Reset() methods

    *ssIntPort   |=  ssIntMask;   // set ss high
    *sckIntPort  |=  sckIntMask;  // set sck high
    *mosiIntPort &= ~mosiIntMask; // set mosi low

    resetModem();
    ////////////////
*/

    beginSPI();
    for(i=0; i<48; i++)
    {
       clockSPI(md[i]);
    }
    endSPI();

    beginSPI();
    clockSPI(PATABLE);           // Power Command
    clockSPI(powerCode);         // And hardcoded for RX
    endSPI();

    beginSPI();
    clockSPI(CHAN);              // Channel Command
    clockSPI(channelCode);
    endSPI();
 
    // For compatibility with Tam Valley Depot Tx
    if (channel_l == 17) {
       beginSPI();
       clockSPI(FREQ2);             // Frequency byte 2 Command
       clockSPI(FREQ2VAL);
       endSPI();
    
       beginSPI();
       clockSPI(FREQ1);             // Frequency byte 1 Command
       clockSPI(FREQ1VAL);
       endSPI();

       beginSPI();
       clockSPI(FREQ0);             // Frequency byte 0 Command
       clockSPI(FREQ0VAL);
       endSPI();

       beginSPI();
       clockSPI(CHAN);              // Channel Command
       clockSPI(CHANVAL);
       endSPI();

       beginSPI();
       clockSPI(DEVIATN);           // Deviatn Command
       clockSPI(deviatnval);
       endSPI();
    }
    if (deviatn_changed) {
       beginSPI();
       clockSPI(DEVIATN);           // Deviatn Command
       clockSPI(deviatnval);
       endSPI();
       deviatn_changed = 0;          // Change flag back
    }
 
    if (mode == TX) {
       strobeSPI(SIDLE); // Ensure Modem is in IDLE for TX
    }
    strobeSPI(mode);                  // TX or RX mode
}


uint8_t strobeSPI(uint8_t data)
{
    beginSPI();

    SPDR = data;              // RX|TX
    while (!(SPSR & SPIFINTMASK));  // wait for byte to clock out

    endSPI();

    return (SPDR);
}

uint8_t readSPI(uint8_t addr)
{
    uint8_t ret;
    
    beginSPI();

    SPDR = addr;              // RX|TX
    while (!(SPSR & SPIFINTMASK));  // wait for byte to clock out
    SPDR = 0;                 // generic out, we only want read back
    while (!(SPSR & SPIFINTMASK));  // wait for byte to clock out
    ret = SPDR;               // readback

    endSPI();

    return (ret);
}

 
