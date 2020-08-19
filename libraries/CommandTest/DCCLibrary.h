#include "Arduino.h"


void SetupDCC(uint8_t DCC_OutPin);


// buffer for command
typedef struct {
    byte data[6]; // Changed from 7
    byte len;
} Message;

