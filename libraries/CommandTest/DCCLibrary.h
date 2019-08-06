#include "Arduino.h"


void SetupDCC(int DCC_OutPin);


// buffer for command
typedef struct {
    byte data[7];
    byte len;
} Message;

