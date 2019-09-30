/*
 * spi.h
 *
 * Created: 12/2/2018 9:33:20 AM
 *  Author: marta
 */ 
 
#ifdef __cplusplus
extern "C" {
#endif

#ifndef SPI_H_
#define SPI_H_

void initializeSPI();
uint8_t sendReceive(uint8_t);
uint8_t readReg(uint8_t addr);
void startModem(uint8_t channel, uint8_t mode);


#endif /* SPI_H_ */
#ifdef __cplusplus
}
#endif