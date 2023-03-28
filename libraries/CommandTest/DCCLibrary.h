#include "Arduino.h"


void SetupDCC(uint8_t DCC_OutPin);


// buffer for command
typedef struct {
    uint8_t Size;
    uint8_t PreambleBits;
    uint8_t Data[6];
} Message;

