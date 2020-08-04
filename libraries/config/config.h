//////////////////////////////////
// Global configuration options //
//////////////////////////////////

// Include only once!
#ifndef config_h
//{

#define config_h

/*
config.h

Created: Sat Mar 14 13:41:01 EDT 2020
Copyright (c) 2020, Darrell Lamm and Martin Sant
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

///////////////////////////////////
//    vvv User Entry Area Below vvv

////////////////////////
// Set band of operation
////////////////////////
// Use ONLY ONE of these
// #define NAEU_900MHz
// #define EU_434MHz
#define NAEU_2p4GHz

//////////////////////////
// Set Transmitter or Receiver
//////////////////////////
// Uncomment ONLY ONE
// For receiver
#define RECEIVE
// For transmitter
// #define TRANSMIT

/////////////////////////////////////////////////
// Set the default channel for NA/EU 900MHz only!
/////////////////////////////////////////////////
// Uncomment ONLY ONE
// To set the default European channel (17) for 900MHz only!
#define EU_DEFAULT
// To set the default to NA channel 0 for 900MHz only!
// #define NA_DEFAULT

//////////////////////////////////////////
// Set the transceiver's crystal frequency
//////////////////////////////////////////
// Uncomment ONLY ONE
// For 27MHz transceivers (e.g., Anaren)
// #define TWENTY_SEVEN_MHZ
// For 26MHz transceiver
#define TWENTY_SIX_MHZ

////////////////////////////////
// Set the LCD's default address
////////////////////////////////
// The LCD display's default address. 
// The address range for TI serial drivers 
// PC8574:  0x20(CCC=LLL) to 0x27(OOO=HHH)(default) and
// PC8574A: 0x38(CCC=LLL) to 0x3F(OOO=HHH)(default)
// O=Open jumper (=High); C=Closed jumper (=Low), 
// addresses are A2,A1,A0 from left to right on the boards
#define LCDADDRESSDEFAULT 0x27

//    ^^^ User Entry Area Above ^^^
///////////////////////////////////



////////////////////////////////
// Determined from defines above
// Do NOT edit below this line
////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
// Messages. Most are simply informational to ensure the correct #define's have been used. 
// Some of the error checking WILL halt compilation with an ERROR.
#define xstr(x) str(x)
#define str(x) #x
// #define DO_PRAGMA(x) _Pragma(str(x))
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// Transmitter or Receiver options
//////////////////////////////////
#if defined(TRANSMIT) && defined (RECEIVE)
//{
#error "ERROR: TRANSMIT and RECEIVE are both defined"
//}
#else
//{
#if ! defined(TRANSMIT) && ! defined(RECEIVE)
#error "ERROR: TRANSMIT are RECEIVE are both undefined"
#endif
//}
#endif

#ifdef TRANSMIT
//{
#pragma message "Info: Compiling for Transmitter"
//}
#else
//{
#ifdef RECEIVE
//{
#pragma message "Info: Compiling for Receiver"
//}
#endif
//}
#endif

////////////////////////////////////////
// Transceiver crystal frequency options
////////////////////////////////////////
#if defined(TWENTY_SEVEN_MHZ) && defined (TWENTY_SIX_MHZ)
//{
#error "ERROR: TWENTY_SEVEN_MHZ and TWENTY_SIX_MHZ are both defined"
//}
#else
//{
#if ! defined(TWENTY_SEVEN_MHZ) && ! defined (TWENTY_SIX_MHZ)
#error "ERROR: TWENTY_SEVEN_MHZ and TWENTY_SIX_MHZ are both undefined"
#endif
//}
#endif

#ifdef TWENTY_SEVEN_MHZ
//{
#pragma message "Info: Compiling for 27MHz transceivers"
//}
#else
//{
#ifdef TWENTY_SIX_MHZ
//{
#pragma message "Info: Compiling for 26MHz transceivers"
//}
#else
//{
#error "Undefined crystal frequency"
//}
#endif
//}
#endif

//////////////////////////
// Band-dependent settings
//////////////////////////
#if ! defined(NAEU_900MHz) && ! defined(EU_434MHz) && ! defined(NAEU_2p4GHz)
#error "ERROR: must define one of the following: NAEU_900MHz, EU_434MHz, and NAEU_2p4GHz"
#endif

#ifdef NAEU_900MHz
//{

#pragma message "Info: Using the EU 869MHz/NA 915MHz ISM band"

#if defined(EU_434MHz) || defined(NAEU_2p4GHz)
#error "ERROR: must define ONLY ONE of the following: NAEU_900MHz, EU_434MHz, and NAEU_2p4GHz"
#endif

#if defined(NA_DEFAULT) && defined (EU_DEFAULT)
//{
#error "ERROR: NA_DEFAULT and EU_DEFAULT are both defined"
//}
#else
//{
#if ! defined(NA_DEFAULT) && ! defined (EU_DEFAULT)
#error "ERROR: NA_DEFAULT and EU_DEFAULT are both undefined"
#endif
//}
#endif

#ifdef NA_DEFAULT
//{
#define CHANNELDEFAULT 0
//}
#else
//{
#ifdef EU_DEFAULT
//{
#define CHANNELDEFAULT 17
//}
#endif
//}
#endif

//}
#else
//{
#ifdef EU_434MHz
//{

#pragma message "Info: Using the EU 434MHz ISM band"

#if defined(NAEU_2p4GHz)
#error "ERROR: must define ONLY ONE of the following: NAEU_900MHz, EU_434MHz, and NAEU_2p4GHz"
#endif

#define CHANNELDEFAULT 0

//}
#else
//{
#ifdef NAEU_2p4GHz
//{

#pragma message "Info: Using the worldwide 2.4GHz ISM band"

#define CHANNELDEFAULT 0

//}
#endif
//}
#endif
//}
#endif

/////////////////
// Misc. settings
/////////////////
#if ! defined(CHANNELDEFAULT)
#error "ERROR: CHANNELDEFAULT is undefined"
#endif

#pragma message "Info: Default channel is " xstr(CHANNELDEFAULT) 

#if ! defined(LCDADDRESSDEFAULT)
#error "ERROR: LCDADDRESSDEFAULT is undefined"
#endif

#pragma message "Info: Default LCD address is " xstr(LCDADDRESSDEFAULT)

///////////////////////////
// End of entire include //
///////////////////////////
//}
#endif
