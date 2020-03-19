// Include only once!
#ifndef config_h
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

//////////////////////////////
//    vvv User Entry Area vvv

// The LAST entry is active!
// To set the default North American channel (0)
#define NA_DEFAULT
// To set the default European channel (17)
#undef NA_DEFAULT


// The LAST entry is active!
// For transmitter
#define TRANSMIT
// For receiver
#undef TRANSMIT

// The LAST entry is active!
// For 26MHz transceiver
#undef TWENTY_SEVEN_MHZ
// For 27MHz transceivers (e.g., Anaren)
#define TWENTY_SEVEN_MHZ

// The LCD display's default address. 
// The address range for TI serial drivers 
// PC8574:  0x20(CCC=LLL) to 0x27(OOO=HHH)(default) and
// PC8574A: 0x38(CCC=LLL) to 0x3F(OOO=HHH)(default)
// O=Open jumper (=High); C=Closed jumper (=Low), 
// addresses are A2,A1,A0 from left to right on the boards
#define LCDADDRESSDEFAULT 0x27

//    ^^^ User Entry Area ^^^^
//////////////////////////////

////////////////////////////////
// Determined from defines above
// Do NOT edit below this line
////////////////////////////////

// Explict RECEIVE define/undef
#ifdef TRANSMIT
#undef RECEIVE
#else
#define RECEIVE
#endif

// Explicit Frequency
#ifdef TWENTY_SEVEN_MHZ
#undef TWENTY_SIX_MHZ
#else
#define TWENTY_SIX_MHZ
#endif

// Messages. When output duing compilation, THESE ARE NOT ERRORS or WARNINGS! They are simply informational.
#define xstr(x) str(x)
#define str(x) #x
#define DO_PRAGMA(x) _Pragma(str(x))

#ifdef TRANSMIT
#pragma message "Info: Compiling for Transmitter"
#endif
#ifdef RECEIVE
#pragma message "Info: Compiling for Receiver"
#endif

#ifdef TWENTY_SEVEN_MHZ
#pragma message "Info: Compiling for 27MHz transceivers"
#endif
#ifdef TWENTY_SIX_MHZ
#pragma message "Info: Compiling for 26MHz transceivers"
#endif

#ifdef NA_DEFAULT
#define CHANNELDEFAULT 0
#pragma message "Info: Default channel is " xstr(CHANNELDEFAULT) " (North America)"
#else
#define EU_DEFAULT
#define CHANNELDEFAULT 17
#pragma message "Info: Default channel is " xstr(CHANNELDEFAULT) " (Europe)"
#endif

#pragma message "Info: Default LCD address is " xstr(LCDADDRESSDEFAULT)

// End of entire include
#endif
