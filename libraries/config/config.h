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
///////////////////////////////////

/////////////////////
// Set the DEBUG Flag
/////////////////////
/* 
Turn on/off Serial output for
the Arduino (or other) serial monitor.
Turning DEBUG on will increase
the size of the firmware, possibly
reducing reliability due to low
residual memory available. Use with
caution or for development only.
*/
#undef DEBUG

////////////////////////
// Set band of operation
////////////////////////
/* Use ONLY ONE #define*/
/* For 896/915MHz EU/NA ISM bands*/
#define NAEU_900MHz
/* For EU-only 434MHz ISM band*/
// #define EU_434MHz
/* For World-Wide 2.4GHz ISM band*/
// #define NAEU_2p4GHz

//////////////////////////////
// Set Transmitter or Receiver
//////////////////////////////
/* Uncomment ONLY ONE #define*/
/* For receiver*/
#define RECEIVER
/* For transmitter*/
// #define TRANSMITTER

/////////////////////////////////////////////////
// Set the default channel for NA/EU 900MHz only!
/////////////////////////////////////////////////
#if defined(NAEU_900MHz)
/* Uncomment ONLY ONE #define*/
/* To set the default to NA channel  0 for 869/915MHz ISM bands only!*/
#define NA_DEFAULT
/* To set the default to EU channel 17 for 869/915MHz ISM bands only!*/
// #define EU_DEFAULT
#endif

//////////////////////////////////////////
// Set the transceiver's crystal frequency
//////////////////////////////////////////
/* Uncomment ONLY ONE #define*/
/* For 27MHz transceivers (e.g., Anaren 869/915MHz (CC110L) and Anaren 869MHz (CC1101) radios)*/
#define TWENTY_SEVEN_MHZ
/* For 26MHz transceiver (almost all other radios, including Anaren 433MHz (CC1101), 915MHz (CC1101), and 2.4GHz (CC2500) radios)*/
// #define TWENTY_SIX_MHZ

//////////////////////////////////////////////////////
// Special settings. You are NOT required to uncomment
//////////////////////////////////////////////////////
/* The final Long address is (CV17-192)*256+CV18*/
// #define AIRMINICV17DEFAULT 230
#if defined(TRANSMITTER)
// #define AIRMINICV18DEFAULT 172
/* For Base Station Repeater Transmitter*/
// #define AIRMINICV18DEFAULT 174
#else
// #define AIRMINICV18DEFAULT 173
/* For Repeater Receiver*/
// #define AIRMINICV18DEFAULT 175
#endif

#if defined(TRANSMITTER)
/* Uncomment for Base Station repeater transmitter and any non-terminal repeater transmitters*/
// #define AUTOIDLEOFFDEFAULT 1
#endif

#if defined(NAEU_900MHz)
/* For 915MHz NA only Repeater transmitters/receivers. Not for European operation!*/
// #define CHANNELDEFAULT 15
#endif

////////////////////////////////////////////////////
// Expert section. Edit only if you really know what
// you are doing. The defaults seem to work well.
////////////////////////////////////////////////////

/* SPCR value -- Change only if you want to alter*/
/* the coms speed and other SPI configuration attributes*/
/* This value sets the communication speed to 512kHz*/
/* bit1 and bit0)*/
// #define SPCRDEFAULT 0x52

/* Do NOT turn off interrupts in "critical" sections dealing with*/
/* assignment of messages for transmission.*/
/* Change at your own risk.*/
// #define DONTTURNOFFINTERRUPTS

/*Test of new 2.4GHz setting*/
// #define ALTERNATIVE2P4

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

////////////////
// Debug options
////////////////
#if defined(DEBUG)
   #pragma message "Info: Compiling debug code, which MAY result in code instability due to low memory."
#endif

//////////////////////////////////
// Transmitter or Receiver options
//////////////////////////////////
#if defined(TRANSMITTER) && defined (RECEIVER)
   #error "ERROR: TRANSMITTER and RECEIVER are both defined"
#elif ! defined(TRANSMITTER) && ! defined(RECEIVER)
   #error "ERROR: TRANSMITTER and RECEIVER are both undefined"
#endif

#if defined(TRANSMITTER)
   #pragma message "Info: Compiling for Transmitter"
#elif defined(RECEIVER)
   #pragma message "Info: Compiling for Receiver"
#endif

////////////////////////////////////////
// Transceiver crystal frequency options
////////////////////////////////////////
#if defined(TWENTY_SEVEN_MHZ) && defined (TWENTY_SIX_MHZ)
   #error "ERROR: TWENTY_SEVEN_MHZ and TWENTY_SIX_MHZ are both defined"
#elif ! defined(TWENTY_SEVEN_MHZ) && ! defined (TWENTY_SIX_MHZ)
   #error "ERROR: TWENTY_SEVEN_MHZ and TWENTY_SIX_MHZ are both undefined"
#endif

#if defined(TWENTY_SEVEN_MHZ)
   #pragma message "Info: Compiling for 27MHz transceivers"
#elif defined(TWENTY_SIX_MHZ)
   #pragma message "Info: Compiling for 26MHz transceivers"
#else
   #error "Undefined crystal frequency"
#endif

//////////////////////////
// Band-dependent settings
//////////////////////////
#if ! defined(NAEU_900MHz) && ! defined(EU_434MHz) && ! defined(NAEU_2p4GHz)
   #error "ERROR: must define one of the following: NAEU_900MHz, EU_434MHz, and NAEU_2p4GHz"
#endif

#if defined(NAEU_900MHz)
//{
   #if defined(EU_434MHz) || defined(NAEU_2p4GHz)
      #error "ERROR: must define ONLY ONE of the following: NAEU_900MHz, EU_434MHz, and NAEU_2p4GHz"
   #endif
   #pragma message "Info: Using the EU 869MHz/NA 915MHz ISM band"
   
   #if defined(NA_DEFAULT) && defined (EU_DEFAULT)
      #error "ERROR: NA_DEFAULT and EU_DEFAULT are both defined"
   #elif ! defined(NA_DEFAULT) && ! defined (EU_DEFAULT)
      #error "ERROR: NA_DEFAULT and EU_DEFAULT are both undefined"
   #endif
   
   #if defined(NA_DEFAULT)
      #if ! defined(CHANNELDEFAULT)
        #define CHANNELDEFAULT 0
      #endif
   #elif  defined(EU_DEFAULT)
      #if ! defined(CHANNELDEFAULT)
        #define CHANNELDEFAULT 17
      #endif
   #endif
//}
#elif defined(EU_434MHz)
//{
   #if defined(NAEU_2p4GHz)
      #error "ERROR: must define ONLY ONE of the following: NAEU_900MHz, EU_434MHz, and NAEU_2p4GHz"
   #endif
   #pragma message "Info: Using the EU 434MHz ISM band"

   #if ! defined(CHANNELDEFAULT)
     #define CHANNELDEFAULT 0
   #endif
//}
#elif defined(NAEU_2p4GHz)
//{
   #pragma message "Info: Using the worldwide 2.4GHz ISM band"
   #if ! defined(CHANNELDEFAULT)
      #define CHANNELDEFAULT 0
   #endif
//}
#endif

/////////////////
// Misc. settings
/////////////////
#if ! defined(CHANNELDEFAULT)
   #error "ERROR: CHANNELDEFAULT is undefined"
#else
   #pragma message "Info: Default channel is " xstr(CHANNELDEFAULT) 
#endif

#if defined(SPCRDEFAULT)
   #pragma message "Info: Changed SPCR value to " xstr(SPCRDEFAULT)
#endif

#if ! defined(DONTTURNOFFINTERRUPTS)
   #pragma message "Info: turning off interrupts in critical-sections"
#else
   #pragma message "Info: NOT turning off interrupts in critical-sections"
#endif

///////////////////////////
// End of entire include //
///////////////////////////
//}
#endif
