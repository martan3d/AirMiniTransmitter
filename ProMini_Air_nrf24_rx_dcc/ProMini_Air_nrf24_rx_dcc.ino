// nrf24 - receiver    12 Mar 2018
#define DEBUG

#ifdef DEBUG
#define PRINT_MAX 129
int print_count = 0;
uint8_t whatChannel;
#endif

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include <avr/io.h>

// use digital pins 6 and 5 for DCC out
// use digital pin D3
#define OUTPUT_PIN PD3

//Timer frequency is 2MHz for ( /8 prescale from 16MHz )
#define TIMER_SHORT 0x8D  // 58usec pulse length
#define TIMER_LONG  0x1B  // 116usec pulse length

byte last_timer = TIMER_SHORT; // store last timer value
byte every_second_isr = 0;  // pulse up or down
byte timer_val = TIMER_SHORT; // The timer value

// definitions for state machine
#define PREAMBLE 0
#define SEPERATOR 1
#define SENDBYTE  2

RF24 radio(9,10);
// RF24 radio(9,10,1000000);
// RF24 radio(9,10,500000);
// RF24 radio(9,10,250000);

const uint64_t pipe00 = 0xE8E8F0F0A0LL;
const uint64_t pipe01 = 0xE8E8F0F0A1LL;
const uint64_t pipe02 = 0xA2LL;  
const uint64_t pipe03 = 0xA3LL;
const uint64_t pipe04 = 0xA4LL;
const uint64_t pipe05 = 0xA5LL;
const uint64_t pipe06 = 0xA6LL;
// const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
byte dataReceived[33];

byte state = PREAMBLE;
byte preamble_count = 16;
byte outbyte = 0;
byte cbit = 0x80;

// buffer for command
typedef struct {
  byte data[6];
  byte len;
} Message;

#define MAXMSG 32

volatile Message msg[MAXMSG] = {
  { {0xFF, 0, 0xFF, 0, 0, 0}, 3},   // idle msg
  { {0xFF, 0, 0xFF, 0, 0, 0}, 3}   //
};

const Message msgIdle =
   {{0xFF, 0, 0xFF, 0, 0, 0}, 3};   // idle msg

volatile uint8_t msgIndex = 0;
volatile uint8_t msgIndexInserted = 0; // runs from 0 to MAXMSG-1
int byteIndex = 0;

void printmsgSerial() {
  Serial.print("rx: loop: msg["); Serial.print(msgIndexInserted,HEX); Serial.print("]:\n");
  Serial.print(" whatChannel: "); Serial.print(whatChannel,DEC); Serial.print("\n");
  Serial.print(" len: "); Serial.print(msg[msgIndexInserted].len,HEX); Serial.print("\n");
  for(byte i=0; i<msg[msgIndexInserted].len; i++) {
     Serial.print(" data["); Serial.print(i,HEX); Serial.print("]: ");
     Serial.print(msg[msgIndexInserted].data[i],HEX); Serial.print("\n");
  }
}

// Output timer
// Setup Timer2.
// Configures the 8-Bit Timer2 to generate an interrupt at the specified frequency.
// Returns the time load value which must be loaded into TCNT2 inside your ISR routine.
void SetupTimer2() {
  //Timer2 Settings: Timer Prescaler /8, mode 0
  //Timmer clock = 16MHz/8 = 2MHz oder 0,5usec
  TCCR2A = 0;
  TCCR2B = 0 << CS22 | 1 << CS21 | 0 << CS20;


  //Timer2 Overflow Interrupt Enable
  TIMSK2 = 1 << TOIE2;

  //load the timer for its first cycle
  TCNT2 = TIMER_SHORT;
}

//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {
  //Capture the current timer value TCTN2. This is how much error we have
  //due to interrupt latency and the work in this function
  //Reload the timer and correct for latency.
  // for more info, see http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/
  byte latency;

  // for every second interupt just toggle signal
  if (every_second_isr)  {

    // PORTD = B01000000;  //use this instead of digitalWrite(6, 1); digitalWrite(5, 0);
    PORTD |= (1<<OUTPUT_PIN);  // OUTPUT_PIN High
    every_second_isr = 0;
    // set timer to last value
    latency = TCNT2;
    TCNT2 = latency + last_timer;

  }  else  {  // != every second interrupt, advance bit or state

    // PORTD = B00100000;  //use this instead of digitalWrite(6, 0); digitalWrite(5, 1);
    PORTD &= ~(1<<OUTPUT_PIN);  // OUTPUT_PIN Low
    every_second_isr = 1;

    switch (state)  {
      case PREAMBLE:
        timer_val = TIMER_SHORT;
        preamble_count--;
        if (preamble_count == 0)  {  // advance to next state
          state = SEPERATOR;
          // get next message
          if (msgIndex != msgIndexInserted) msgIndex = (msgIndex+1) % MAXMSG;
          else {// If no new message, send an idle message in the updated msgIndexInserted slot
             msgIndexInserted = (msgIndexInserted+1) % MAXMSG;
             msgIndex = msgIndexInserted;
             memcpy((void *)&msg[msgIndexInserted], (void *)&msgIdle, sizeof(Message)); // copy the idle message
          }
#if 0
     if (!print_count) printmsgSerial();
     print_count = (print_count+1) % PRINT_MAX;
#endif
          byteIndex = 0; //start msg with byte 0
        }
        break;
      case SEPERATOR:
        timer_val = TIMER_LONG;
        // then advance to next state
        state = SENDBYTE;
        // goto next byte ...
        cbit = 0x80;  // send this bit next time first
        outbyte = msg[msgIndex].data[byteIndex];
        break;
      case SENDBYTE:
        if (outbyte & cbit)  {
          timer_val = TIMER_SHORT;
        }  else  {
          timer_val = TIMER_LONG;
        }
        cbit = cbit >> 1;
        if (cbit == 0)  {  // last bit sent, is there a next byte?
          byteIndex++;
          if (byteIndex >= msg[msgIndex].len)  {
            // this was already the XOR byte then advance to preamble
            state = PREAMBLE;
            preamble_count = 16;
          }  else  {
            // send separtor and advance to next byte
            state = SEPERATOR ;
          }
        }
        break;
    } // end of switch

    // Set up output timer
    latency = TCNT2;
    TCNT2 = latency + timer_val;
    last_timer = timer_val;


  } // end of else ! every_seocnd_isr

}

void setup(void)
{
#ifdef DEBUG
   Serial.begin(115200);
#endif

   // DDRD |= B01100000;   //  register D5 for digital pin 5, D6 for digital pin 6
   DDRD |= (1<<OUTPUT_PIN); //  register OUTPUT_PIN for Output

   SetupTimer2();
#ifdef DEBUG
   Serial.println("rx: setup: timer2 ready");
#endif

   radio.begin();
#ifdef DEBUG
   if (radio.isChipConnected()) Serial.print("Chip is connected\n");
   else Serial.print("Chip is NOT connected\n");
#endif
   radio.setChannel(10);
   // radio.setDataRate( RF24_250KBPS );
   radio.setDataRate(RF24_1MBPS);
   radio.enableDynamicPayloads();
   radio.enableDynamicAck(); // Added. Not sure it's necessary on the rx side
   radio.openWritingPipe(pipe00);
   radio.openReadingPipe(1, pipe01);
   radio.startListening();

#ifdef DEBUG
   Serial.println("rx: setup: radio ready");
#endif
   delay(5000);
}


void loop(){

  // delay(100);
  // while(!radio.available());
  uint8_t len;
  if ( radio.available(&whatChannel) ) {
     // radio.read( &dataReceived, sizeof(dataReceived) );
     len = radio.getDynamicPayloadSize();
     radio.read( &dataReceived, len );
     noInterrupts();
     msgIndexInserted = (msgIndexInserted+1) % MAXMSG;
     msg[msgIndexInserted].len = dataReceived[0];
     // for(byte i=0; i<msg[msgIndexInserted].len; i++) msg[msgIndexInserted].data[i]=dataReceived[i+1];
     memcpy((void *)&msg[msgIndexInserted].data[0],(void *)&dataReceived[1],dataReceived[0]);
     interrupts();
#ifdef DEBUG
     if (!print_count) printmsgSerial();
     print_count = (print_count+1) % PRINT_MAX;
#endif
  }

}
