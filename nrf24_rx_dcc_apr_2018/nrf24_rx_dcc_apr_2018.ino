  // nrf24 - receiver    12 Mar 2018
#define DEBUG

#ifdef DEBUG
#define PRINT_MAX 64
int print_count = 0;
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
byte flag = 0; // used for short or long pulse
byte every_second_isr = 0;  // pulse up or down

// definitions for state machine
#define PREAMBLE 0
#define SEPERATOR 1
#define SENDBYTE  2

// RF24 radio(9,10);
// RF24 radio(9,10,1000000);
RF24 radio(9,10,500000);
// RF24 radio(9,10,250000);

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
byte dataReceived[32];

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
        flag = 1; // short pulse
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
          byteIndex = 0; //start msg with byte 0
        }
        break;
      case SEPERATOR:
        flag = 0; // long pulse
        // then advance to next state
        state = SENDBYTE;
        // goto next byte ...
        cbit = 0x80;  // send this bit next time first
        outbyte = msg[msgIndex].data[byteIndex];
        break;
      case SENDBYTE:
        if (outbyte & cbit)  {
          flag = 1;  // send short pulse
        }  else  {
          flag = 0;  // send long pulse
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
    if (flag)  {  // if data==1 then short pulse
      latency = TCNT2;
      TCNT2 = latency + TIMER_SHORT;
      last_timer = TIMER_SHORT;
    }  else  {   // long pulse
      latency = TCNT2;
      TCNT2 = latency + TIMER_LONG;
      last_timer = TIMER_LONG;
    }

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
   radio.openReadingPipe(1, thisSlaveAddress);
   radio.startListening();

#ifdef DEBUG
   Serial.println("rx: setup: radio ready");
#endif
   delay(5000);
}

void printmsgSerial() {
  Serial.print("rx: loop: msg["); Serial.print(msgIndexInserted,HEX); Serial.print("]:\n");
  Serial.print(" len: "); Serial.print(msg[msgIndexInserted].len,HEX); Serial.print("\n");
  for(byte i=0; i<msg[msgIndexInserted].len; i++) {
     Serial.print(" data["); Serial.print(i,HEX); Serial.print("]: ");
     Serial.print(msg[msgIndexInserted].data[i],HEX); Serial.print("\n");
  }
}

void loop(){

  // delay(100);
  // while(!radio.available());
  if ( radio.available() ) {
     radio.read( &dataReceived, sizeof(dataReceived) );
     noInterrupts();
     msgIndexInserted = (msgIndexInserted+1) % MAXMSG;
     msg[msgIndexInserted].len = dataReceived[0];
     for(byte i=0; i<msg[msgIndexInserted].len; i++) msg[msgIndexInserted].data[i]=dataReceived[i+1];
     interrupts();
#ifdef DEBUG
     if (!print_count) printmsgSerial();
     print_count = (print_count+1) % PRINT_MAX;
#endif
  }

}
