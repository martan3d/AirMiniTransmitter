// nrf24 - transmit DCC data 1 April 2018
#undef DEBUG

#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include <SPI.h>    
#include <NmraDcc.h>
#include <avr/io.h>

// Actual input pin. No conversion do PD3! Dcc.init does this.
#define INPUT_PIN 2

//Settings
RF24 radio(9,10);
// RF24 radio(9,10,1000000);
// RF24 radio(9,10,500000);
NmraDcc Dcc ;

uint8_t payload[33]={3,0xFF,0,0xFF}; // initialize with idle message

// const byte slaveAddress[5] = {'R','x','A','A','A'};
const uint64_t pipe00 = 0xE8E8F0F0A0LL;
const uint64_t pipe01 = 0xE8E8F0F0A1LL;
const uint64_t pipe02 = 0xE8E8F0F0A2LL;  
const uint64_t pipe03 = 0xE8E8F0F0A3LL;
const uint64_t pipe04 = 0xE8E8F0F0A4LL;
const uint64_t pipe05 = 0xE8E8F0F0A5LL;
const uint64_t pipe06 = 0xE8E8F0F0A6LL;

void printMsgSerial() {
  Serial.print("tx: notifyDccMsg: payload(Msg):\n"); 
  Serial.print(" Size: "); Serial.print(payload[0],HEX); Serial.print("\n");
  for(uint8_t i=0; i<payload[0]; i++) {
     Serial.print(" Data[: "); Serial.print(i,HEX); Serial.print("]: ");
     Serial.print(payload[i+1],HEX); Serial.print("\n");
  }
}

extern void notifyDccMsg( DCC_MSG * Msg ) {
    // noInterrupts(); 
    payload[0] = Msg->Size;
    // for(uint8_t i=0; i<payload[0]; i++) payload[i+1] = Msg->Data[i];
    memcpy((void *)&payload[1],(void *)&Msg->Data[0],Msg->Size);
    // interrupts();
    radio.write( payload, payload[0]+1, 1 ); // NOACK
#ifdef DEBUG
    printMsgSerial();
#endif
}

void setup() {
#ifdef DEBUG
   Serial.begin(115200);
#endif

   radio.begin();
#ifdef DEBUG
   if (radio.isChipConnected()) Serial.print("The chip is connected\n");
   else Serial.print("The chip is NOT connected\n");
#endif
   // radio.setDataRate( RF24_250KBPS );
   radio.setDataRate(RF24_1MBPS);
   radio.setChannel(10); 
   radio.setRetries(0,0); // delay, count
   radio.enableDynamicPayloads();
   radio.enableDynamicAck(); // Added
   radio.setPALevel(RF24_PA_MIN,0);
   radio.openReadingPipe(0,pipe00);
   radio.openWritingPipe(pipe01);
   // radio.stopListening();
    
#ifdef DEBUG
   Serial.println("tx: setup: radio ready");
#endif

   // Dcc.pin(0, 2, 0);
   Dcc.pin(0, INPUT_PIN, 0);
   // Call the main DCC Init function to enable the DCC Receiver
   Dcc.init( MAN_ID_DIY, 100,   FLAGS_DCC_ACCESSORY_DECODER, 0 ); 

#ifdef DEBUG
   Serial.println("tx: setup: Dcc ready");
#endif
   delay(5000);
}

void loop(){
   Dcc.process();
}
