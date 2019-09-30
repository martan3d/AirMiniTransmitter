/*
 * servotimer.h
 *
 * Created: 11/3/2017 7:24:28 PM
 *  Author: martan
 */ 

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SERVOTIMER_H_
#define SERVOTIMER_H_

#define F_CPU  16000000

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

#ifdef __cplusplus
}
#endif
