/*
 * schedule.c
 *
 * Created: 11/20/2018 5:22:19 PM
 *  Author: martan
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









