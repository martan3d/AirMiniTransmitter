// DCC Led Accessory Decoder with CV support
// Luca Dentella, 29.12.2018
 
 
// include NmraDcc, EEPROM and Timer1 libraries
#include <NmraDcc.h>
#include <EEPROM.h>
#include <TimerOne.h>
 
// definitions
#define DCC_PIN         2
#define PAIR_ADDRESS    1
#define GREEN_LED_PIN   3
#define RED_LED_PIN     4
 
// CVs in EEPROM
#define CV_ACCESSORY_DECODER_MODE        10
#define CV_ACCESSORY_DECODER_BLINK_FREQ  11

// global variables
NmraDcc Dcc;
int decoderMode;
int blinkFrequency;
int ledActive;
 
 
void blinkLED() {
 
  digitalWrite(ledActive, !digitalRead(ledActive));
}
 
 
void notifyCVChange(uint16_t CV, uint8_t Value) {

  Serial.print("CV ");
  Serial.print(CV, DEC);
  Serial.print(" changed, new value = ");
  Serial.println(Value, DEC);

  if(CV == CV_ACCESSORY_DECODER_MODE) decoderMode = Value;
  else if(CV == CV_ACCESSORY_DECODER_BLINK_FREQ) blinkFrequency = Value;
}
 
 
void notifyDccAccState(uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State) {
  int pairAddress = (OutputAddr >> 1) + 1;
  int outputInPair = OutputAddr & 0x01;
 
  Serial.print("* new accessory decoder packet received (BoardAddr=");
  Serial.print(BoardAddr, DEC);
  Serial.print(", pairAddress=");
  Serial.print(pairAddress, DEC);
  Serial.print(", outputInPair=");
  Serial.println(outputInPair, DEC);
 
  // check if the command is for our address and output
  if((BoardAddr == Dcc.getAddr()) && (pairAddress == PAIR_ADDRESS)) {
 
    // disable a previous interrupt
    Timer1.detachInterrupt();
 
    // turn off the output and select the correct led to turn on/blink
    if(outputInPair == 0) {     
      digitalWrite(GREEN_LED_PIN, LOW);
      ledActive = RED_LED_PIN;
    } else {
      digitalWrite(RED_LED_PIN, LOW);
      ledActive = GREEN_LED_PIN;   
    }
 
    // if mode is blink, start the timer, otherwise turn the led on
    if(decoderMode == 1) {
      Timer1.setPeriod(5000000 / blinkFrequency);
      Timer1.attachInterrupt(blinkLED);
    }
    else digitalWrite(ledActive, HIGH);
  }
}
 
 
void setup() {
 
  // Serial output for debugging
  Serial.begin(115200);
  Serial.println("DCC Led Accessory Decoder with CVs");
  Serial.println();
 
  // init NmraDcc library (PIN, manufacturer, version...)
  Dcc.pin(digitalPinToInterrupt(DCC_PIN), DCC_PIN, 1);
  Dcc.init(MAN_ID_DIY, 1, FLAGS_DCC_ACCESSORY_DECODER, 0);
  Serial.println("- NmraDcc library initialized");
 
  // init Timer1 library
  Timer1.initialize(5000000);
  Serial.println("- Timer1 library initialized");
 
  // configure pins
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  Serial.println("- pins configured");
 
  decoderMode = Dcc.getCV(CV_ACCESSORY_DECODER_MODE);
  blinkFrequency = Dcc.getCV(CV_ACCESSORY_DECODER_BLINK_FREQ);
  Serial.println("- CV values set");
  Serial.println();
 
  Serial.println("Decoder ready:");
  Serial.print("* Address = "); Serial.println(Dcc.getAddr());
  Serial.print("* Decoder mode = "); Serial.println(decoderMode);
  Serial.print("* blinkFrequency = "); Serial.println(blinkFrequency);
  Serial.println();
}
 
 
void loop() {
 
   Dcc.process();
}
