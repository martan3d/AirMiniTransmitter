#include "Arduino.h"


void SetupDCC(int DCC_OutPin);


// buffer for command
typedef struct {
    byte data[6]; // Changed from 7
    byte len;
} Message;

