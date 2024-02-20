/* 
ESP32_DCC.ino 

Copyright (c) 2021-2023, Darrell Lamm
All rights reserved.

Test program to exercize the ESP32 to receive DCC
and print it out for verification via the serial
port.

Redistribution and use in source and binary forms, with or
without modification, are permitted provided that the following
conditions are met: 
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define DCC_PIN    2


#include <NmraDcc.h>

DCC_MSG msg;
NmraDcc DCC;

extern void notifyDccMsg(DCC_MSG *Msg) {
  if ((3 <= Msg->Size) && (Msg->Size <= 6)) {  // Check for a valid message
     memcpy((void *)&msg, (void *)Msg, sizeof(DCC_MSG));
  }
  printMsgSerial();
}  // End of notifyDccMsg

void printMsgSerial() {
  Serial.print("msg:\n");
  Serial.print(" len: ");
  Serial.print(msg.Size, HEX);
  Serial.print("\n");
  for (uint8_t i = 0; i < msg.Size; i++) {
     Serial.print(" data[");
     Serial.print(i, HEX);
     Serial.print("]: ");
     Serial.print(msg.Data[i], HEX);
     Serial.print("\n");
  }
}

void setup() {

  Serial.begin(115200);
  uint8_t maxWaitLoops = 255;
  while(!Serial && maxWaitLoops--)
     delay(20);

  Serial.println("ESP32 Decoder");
  Serial.println("DCC messages should follow!");

  // Set up the input and output pins


  DCC.pin(DCC_PIN, 1);

  // Call the main DCC Init function to enable the DCC Receiver
  DCC.init(MAN_ID_DIY, 100,   FLAGS_DCC_ACCESSORY_DECODER, 0);

}  // End of setup

void loop() {

  DCC.process();  // The DCC library does it all with the callback notifyDccMsg!

}  // end of loop

////////////////////////////
// End of file ESP32_DCC.ino
////////////////////////////
