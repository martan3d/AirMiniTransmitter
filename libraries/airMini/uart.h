/*
 * uart.h
 *
 * Created: 7/14/2016 7:50:31 PM
 *  Author: martan
 */ 

#ifdef __cplusplus
extern "C" {
#endif

#ifndef UART_H_
#define UART_H_

#define F_CPU  16000000

void initUART(long baud);
void SendByte(uint8_t c);

#endif /* UART_H_ */

#ifdef __cplusplus
}
#endif
