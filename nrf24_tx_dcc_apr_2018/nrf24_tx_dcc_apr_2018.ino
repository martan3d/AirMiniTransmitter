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
// RF24 radio(9,10);
// RF24 radio(9,10,1000000);
RF24 radio(9,10,500000);
NmraDcc Dcc ;

uint8_t payload[32]={3,0xFF,0,0xFF}; // initialize with idle message

const byte slaveAddress[5] = {'R','x','A','A','A'};

void setup() {
#if DEBUG
   Serial.begin(115200);
#endif

   radio.begin();
#if DEBUG
   if (radio.isChipConnected()) Serial.print("The chip is connected\n");
   else Serial.print("The chip is NOT connected\n");
#endif
   // radio.setDataRate( RF24_250KBPS );
   radio.setDataRate(RF24_1MBPS);
   radio.setChannel(10); 
   radio.setRetries(0,0); // delay, count
   radio.enableDynamicPayloads();
   radio.enableDynamicAck(); // Added
   radio.setPALevel(RF24_PA_MAX,0);
   radio.openWritingPipe(slaveAddress);
   radio.stopListening();
    
#if DEBUG
   Serial.println("tx: setup: radio ready");
#endif

   // Dcc.pin(0, 2, 0);
   Dcc.pin(0, INPUT_PIN, 0);
   // Call the main DCC Init function to enable the DCC Receiver
   Dcc.init( MAN_ID_DIY, 100,   FLAGS_DCC_ACCESSORY_DECODER, 0 ); 

#if DEBUG
   Serial.println("tx: setup: Dcc ready");
#endif
   delay(5000);
}

void printMsgSerial() {
  Serial.print("tx: notifyDccMsg: payload(Msg):\n"); 
  Serial.print(" Size: "); Serial.print(payload[0],HEX); Serial.print("\n");
  for(uint8_t i=0; i<payload[0]; i++) {
     Serial.print(" Data[: "); Serial.print(i,HEX); Serial.print("]: ");
     Serial.print(payload[i+1],HEX); Serial.print("\n");
  }
}

void loop(){
   Dcc.process();
}

extern void notifyDccMsg( DCC_MSG * Msg ) {
    // noInterrupts(); 
    payload[0] = Msg->Size;
    for(uint8_t i=0; i<payload[0]; i++) payload[i+1] = Msg->Data[i];
    // interrupts();
    radio.write( payload, payload[0]+1, 1 ); // NOACK
#if DEBUG
    printMsgSerial();
#endif
}
