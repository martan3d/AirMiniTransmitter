/*
servotimer.h

Created: 11/3/2017 7:24:28 PM
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

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef SERVOTIMER_H_
#define SERVOTIMER_H_

// #define F_CPU  16000000

void initServoTimer();
void setServoPulse(uint8_t i, int16_t pulse);
void delay_us(uint32_t t);
uint64_t getMsClock();
int16_t getWatchDog();
void resetWatchDog(int16_t value);

void setServoDefault0(int16_t sd);
void setServoDefault1(int16_t sd);
void setServoDefault2(int16_t sd);

void setServoLow0(int16_t lo);
void setServoLow1(int16_t lo);
void setServoLow2(int16_t lo);

void setServoHigh0(int16_t hi);
void setServoHigh1(int16_t hi);
void setServoHigh2(int16_t hi);

void setServoReverseValue(uint8_t direction);

#endif /* SERVOTIMER_H_ */

#if defined(__cplusplus)
}
#endif
