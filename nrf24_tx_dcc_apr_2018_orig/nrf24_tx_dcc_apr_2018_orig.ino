  // nrf24 - transmit DCC data 1 April 2018

#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include "SPI.h"    
#include "NmraDcc.h"

//Settings
 RF24 radio(9,10);
 NmraDcc  Dcc ;
 DCC_MSG  Packet ;
 struct CVPair{
  uint16_t  CV;
  uint8_t   Value; };


const byte slaveAddress[5] = {'R','x','A','A','A'};
char dataToSend[20] = "DD,9,63,0,54,";

void setup() {
  Serial.begin(115200);
    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.setChannel(10); 
    radio.setRetries(0,0); // delay, count
    radio.openWritingPipe(slaveAddress);
    
  Serial.println("Ready");

  Dcc.pin(0, 2, 0);
// Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 100,   FLAGS_DCC_ACCESSORY_DECODER , 0 ); 
  Serial.println("Ready");
}

void loop(){
Dcc.process();
}

extern void notifyDccMsg( DCC_MSG * Msg ) {
 if( ( Msg->Data[0] == 0 ) && ( Msg->Data[1] == 0 ) ) return;  //reset packlet
 if( ( Msg->Data[0] == 0b11111111 ) && ( Msg->Data[1] == 0 ) ) return;  //idle packet   
String s1 = String(Msg->Data[0], DEC) + ",";
String s2 = String(Msg->Data[1], DEC) + ",";
String s3 = String(Msg->Data[2], DEC) + ",";
String s4 = String(Msg->Data[3], DEC) + ",";
String Message = "DD," + s1 + s2 + s3 + s4 + '\0';

   int Msg_size = Message.length();
  for (int i = 0; i <= Msg_size; i++) {
    dataToSend[i] = Message.charAt(i);
  }
    Serial.println(dataToSend);
    radio.write( &dataToSend, sizeof(dataToSend) );
}

