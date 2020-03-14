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

// The LAST entry is active!
// For US/Canadian use
#define FCC_IC_ISM
// For European use
#undef FCC_IC_ISM



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

// Explicit CE undef/define
#ifdef FCC_IC_ISM
#undef ETSI_ISM
#else
#define ETSI_ISM
#endif

// Explicit Frequency
#ifdef TWENTY_SEVEN_MHZ
#undef TWENTY_SIX_MHZ
#else
#define TWENTY_SIX_MHZ
#endif

// Messages
#ifdef TRANSMIT
#warning "Note: Compiling for Transmitter"
#endif
#ifdef RECEIVE
#warning "Note: Compiling for Receiver"
#endif

#ifdef TWENTY_SEVEN_MHZ
#warning "Note: Compiling for 27MHz transceivers"
#endif
#ifdef TWENTY_SIX_MHZ
#warning "Note: Compiling for 26MHz transceivers"
#endif

#ifdef FCC_IC_ISM
#warning "Note: Compiling for North American use"
#endif
#ifdef ETSI_ISM
#warning "Note: Compiling for European use"
#endif

