/*
 * schedule.h
 *
 * Created: 11/20/2018 5:21:22 PM
 *  Author: martan
 */ 
 
#ifdef __cplusplus
extern "C" {
#endif

#ifndef SCHEDULE_H_
#define SCHEDULE_H_

#define TASK0           0x80
#define TASK1           0x40
#define TASK2           0x20
#define TASK3           0x10
#define TASK4           0x08
#define TASK5           0x04
#define TASK6           0x02
#define IDLE            0x01

uint8_t masterSchedule(void);
void setScheduledTask(uint8_t sb);
void clearScheduledTask(uint8_t sb);


#endif /* SCHEDULE_H_ */

#ifdef __cplusplus
}
#endif