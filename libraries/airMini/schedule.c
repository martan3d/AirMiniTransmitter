/*
schedule.c

Created: 11/20/2018 5:22:19 PM
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

#include <avr/io.h>
#include "schedule.h"

/* 
 *
 * Priority Background Scheduler 
 * 
 * Interrupt routines set flags to run Background tasks
 * Timed priority and high priority is implemented
 * in the main.c background loop
 *
 */

uint8_t scheduleByte = 0;

uint8_t masterSchedule(void)
{
    uint8_t choice = 0;
    // x x x x  x x x x                                         // These match with switch in main.c
    // | | | |  | | | |
    // | | | |  | | | ------ Idle                   :0          // Lowest Priority
    // | | | |  | | -------- Task 6                 :1
    // | | | |  | ---------- Task 5                 :2
    // | | | |  ------------ Task 4                 :3
    // | | | --------------- Task 3                 :4
    // | | ----------------- Task 2                 :5
    // | | ----------------- Task 1                 :6
    // | ------------------- Task 0                 :7          // Highest Priority

    if(scheduleByte & 0x01) choice = IDLE;
    if(scheduleByte & 0x02) choice = TASK6;
    if(scheduleByte & 0x04) choice = TASK5;
    if(scheduleByte & 0x08) choice = TASK4;
    if(scheduleByte & 0x10) choice = TASK3;
    if(scheduleByte & 0x20) choice = TASK2;
    if(scheduleByte & 0x40) choice = TASK1;
    if(scheduleByte & 0x80) choice = TASK0;

    return choice;
}

void setScheduledTask(uint8_t sb)
{
    scheduleByte |= sb;
}

void clearScheduledTask(uint8_t sb)
{
    scheduleByte &= ~(sb);
}









