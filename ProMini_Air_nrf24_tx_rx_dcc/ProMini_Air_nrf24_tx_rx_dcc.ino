// ProMini_Air_nrf24_tx_rx_dcc.ino - transmit/receive DCC date 30 May 2021
//////////////////////////////////////////////////////////////////////////////////////////
// For compilation output messages. Most are simply informational to ensure the correct 
// #define's have been used. 
// Some of the error checking WILL halt compilation with an ERROR.
#define xstr(x) str(x)
#define str(x) #x
// #define DO_PRAGMA(x) _Pragma(str(x))
//////////////////////////////////////////////////////////////////////////////////////////

#define TRANSMITTER
#define DEBUG
#undef DEBUG

#if defined(TRANSMITTER)
#undef RECEIVER
#else
#define RECEIVER
#endif

#if defined(RECEIVER)
#undef TRANSMITTER
#endif

//////////////////////////////////
// Transmitter or Receiver options
//////////////////////////////////
#if defined(TRANSMITTER) && defined (RECEIVER)
   #error "ERROR: TRANSMITTER and RECEIVER are both defined"
#elif ! defined(TRANSMITTER) && ! defined(RECEIVER)
   #error "ERROR: TRANSMITTER and RECEIVER are both undefined"
#endif

#if defined(TRANSMITTER)
   #pragma message "Info: Compiling for Transmitter"
#elif defined(RECEIVER)
   #pragma message "Info: Compiling for Receiver"
#endif

#if defined(TRANSMITTER)
#define USE_LCD
#elif defined(RECEIVER)
#define USE_LCD
#endif


///////////////
// Libraries //
///////////////

#include <SPI.h>    
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include <avr/io.h>
#include <NmraDcc.h>

#if defined(USE_LCD)
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#endif

/////////////////////////////////////
// Global variable and definitions //
/////////////////////////////////////

// Timing
#define EEPROMDELAYMS         100 // Delay after eeprom write in ms
#define MILLISEC          1000ULL //   1ms, using micros (usec)
#define QUARTERSEC      250000ULL // 250ms, using micros (usec)
#define SEC            1000000ULL //    1s, using micros (usec)
#define BACKGROUNDTIME    8000ULL //   8ms, using micros (usec)

#if defined(TRANSMITTER)
uint8_t initialWait = 0;                     // Initial wait status for receiving valid DCC
#else
uint8_t initialWait = 0;                     // Initial wait status for receiving valid DCC
#endif

enum {INITIAL, INFO, NONE} LCDwhichBanner = INITIAL;

uint16_t CVnum;                              // CV numbers consume 10 bits
uint8_t CVval;                               // CV values consume only 8 bits


#define ISSET 0b10101010

uint8_t SET_DEFAULT = 1;
uint8_t EEMEM EEFirst;  // Store the first time

// Channel-related
#define CHANNELDEFAULT 10          // 
#define CHANNELS_MAX 10            // 
uint8_t CHANNEL = CHANNELDEFAULT;  // 
uint8_t EEMEM EEisSetCHANNEL;      // Stored RF channel is set
uint8_t EEMEM EECHANNEL;           // Stored RF channel #
// uint8_t EEMEM EECHANNELDefault; // Stored RF channel #

// Power-related
#define POWERLEVELDEFAULT RF24_PA_MIN
uint8_t powerLevel = POWERLEVELDEFAULT; // The modem power level (>=0 and <=10). Communicated to spi.c
uint8_t EEMEM EEisSetpowerLevel;        // Stored DC output power level is set
uint8_t EEMEM EEpowerLevel;             // Stored DC output power level 
// uint8_t EEMEM EEpowerLevelDefault;   // Stored DC output power level 

#if defined(RECEIVER)
// Filtering-related
#define FILTERMODEMDATADEFAULT 0
uint8_t filterModemData = FILTERMODEMDATADEFAULT; // Set the logical for whether to always use modem data.
uint8_t EEMEM EEisSetfilterModemData;             // Stored filter modem data set flag
uint8_t EEMEM EEfilterModemData;                  // Stored filter modem data in ms
// uint8_t  EEMEM EEfilterModemDataDefault;       // Stored filter modem data in ms
#endif

// CV-Related
#define AIRMINICV1DEFAULT 3

#if ! defined(AIRMINICV17DEFAULT)
#define AIRMINICV17DEFAULT 227
#endif
#pragma message "Info: Default CV17 is " xstr(AIRMINICV17DEFAULT)

#if ! defined(AIRMINICV18DEFAULT)
//{
#if defined(TRANSMITTER)
#define AIRMINICV18DEFAULT 40
#else
#define AIRMINICV18DEFAULT 41
#endif
//}
#endif
#pragma message "Info: Default CV18 is " xstr(AIRMINICV18DEFAULT)

#define AIRMINICV29DEFAULT 32

uint8_t AirMiniCV1;                   // The AirMini's address, HIGH byte
uint8_t EEMEM EEisSetAirMiniCV1;      // Stored AirMini decoder short address is set
uint8_t EEMEM EEAirMiniCV1;           // Stored AirMini decoder short address
// uint8_t EEMEM EEAirMiniCV1Default; // Stored AirMini decoder short address

uint8_t AirMiniCV17;                   // The AirMini's address, HIGH byte
uint8_t AirMiniCV17tmp;                // The AirMini's address, HIGH byte, temporary value until CV18 is reassigned in ops mode
uint8_t EEMEM EEisSetAirMiniCV17;      // Stored AirMini decoder high byte address is set
uint8_t EEMEM EEAirMiniCV17;           // Stored AirMini decoder high byte address
// uint8_t EEMEM EEAirMiniCV17Default;

uint8_t AirMiniCV18;                   // The AirMini's address, LOW byte
uint8_t EEMEM EEisSetAirMiniCV18;      // Stored AirMini decoder low byte address is set
uint8_t EEMEM EEAirMiniCV18;           // Stored AirMini decoder low byte address
// uint8_t EEMEM EEAirMiniCV18Default; // Stored AirMini decoder low byte address

uint8_t AirMiniCV29;                   // The AirMini's address, HIGH byte
uint8_t AirMiniCV29Bit5;               // The value of AirMiniCV29, bit 5
uint8_t EEMEM EEisSetAirMiniCV29;      // Stored AirMini decoder configuration variable is set
uint8_t EEMEM EEAirMiniCV29;           // Stored AirMini decoder configuration variable
// uint8_t EEMEM EEAirMiniCV29Default; // Stored AirMini decoder configuration variable

enum {ACCEPTED, IGNORED, PENDING} CVStatus = ACCEPTED;

// Declarations
uint8_t payload[33]={3,0xFF,0,0xFF}; // initialize with idle message

uint64_t now;
uint64_t then;

// DCC_MSG type defined in NmraDcc.h
volatile DCC_MSG *dccptr;
uint8_t sendbuffer[sizeof(DCC_MSG)];
uint8_t modemCVResetCount=0;
uint8_t dccptrAirMiniCVReset[sizeof(DCC_MSG)];
uint8_t dccptrNULL[sizeof(DCC_MSG)];
uint8_t dccptrRepeatCount = 0;
uint8_t dccptrRepeatCountMax = 2;
uint8_t msgReplaced = 0;
uint8_t tmpuint8 = 0;
uint8_t do_not_filter = 0;
uint8_t countPtr = 1;

#if defined(TRANSMITTER)
// TRANSMITTER
//////////////

// Actual input pin. No conversion do PD3! Dcc.init does this.
//#define INPUT_PIN 2
//#define EXTINT_NUM 0

#define INPUT_PIN 3
#define EXTINT_NUM 1

//////////////
// TRANSMITTER
#else
// RECEIVER
///////////

uint8_t whatChannel;
#ifdef DEBUG
#define PRINT_MAX 129
int print_count = 0;
#endif

// use digital pins 6 and 5 for DCC out
// use digital pin D3
#define OUTPUT_PIN PD3

///////////
// RECEIVER
#endif

//Settings for both TX/RX
RF24 radio(9,10); // CE,CSN

#if defined(TRANSMITTER)
// TRANSMITTER
//////////////

NmraDcc Dcc;

//////////////
// TRANSMITTER
#else
// RECEIVER
///////////

//Timer frequency is 2MHz for ( /8 prescale from 16MHz )
#define TIMER_SHORT 0x8D  // 58usec pulse length
#define TIMER_LONG  0x1B  // 116usec pulse length


// definitions for state machine
byte last_timer = TIMER_SHORT; // store last timer value
byte timer_val = TIMER_LONG; // The timer value
byte every_second_isr = 0;  // pulse up or down

enum {PREAMBLE, SEPERATOR, SENDBYTE} state = PREAMBLE;
byte preamble_count = 16;
byte outbyte = 0;
byte cbit = 0x80;
int byteIndex = 0;

///////////
// RECEIVER
#endif

#define MAXMSG 32

volatile DCC_MSG msg[MAXMSG] = {
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   // idle msg
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}}   //
};

const DCC_MSG msgIdle =
   {3, 16, {0xFF, 0, 0xFF, 0, 0, 0}};   // idle msg

volatile uint8_t msgIndex = 0;
volatile uint8_t msgIndexInserted = 0; // runs from 0 to MAXMSG-1

#if defined(TRANSMITTER)
// TRANSMITTER
//////////////

// const byte slaveAddress[5] = {'R','x','A','A','A'};
const uint64_t pipe00 = 0xE8E8F0F0A0ULL;
const uint64_t pipe01 = 0xE8E8F0F0A1ULL;
const uint64_t pipe02 = 0xE8E8F0F0A2ULL;  
const uint64_t pipe03 = 0xE8E8F0F0A3ULL;
const uint64_t pipe04 = 0xE8E8F0F0A4ULL;
const uint64_t pipe05 = 0xE8E8F0F0A5ULL;
const uint64_t pipe06 = 0xE8E8F0F0A6ULL;

//////////////
// TRANSMITTER
#else
// RECEIVER
///////////

// const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
const uint64_t pipe00 = 0xE8E8F0F0A0ULL;
const uint64_t pipe01 = 0xE8E8F0F0A1ULL;
const uint64_t pipe02 = 0xA2ULL;  
const uint64_t pipe03 = 0xA3ULL;
const uint64_t pipe04 = 0xA4ULL;
const uint64_t pipe05 = 0xA5ULL;
const uint64_t pipe06 = 0xA6ULL;

///////////
// RECEIVER
#endif

#if defined(USE_LCD)
uint8_t printDCC = 1;                        // Global flag for LCD for DCC msg display
const char *bannerString = "ProMini Air NRF";
bool lcdInitialized = false;
uint8_t LCDAddress;                            // The I2C address of the LCD
bool LCDFound = false;                         // Whether a valid lcd was found
#define LCDCOLUMNS 16                          // Number of LCD columns
#define LCDROWS 2                              // Number of LCD rows 
uint64_t LCDTimePeriod=2ULL*SEC;               // Set up the LCD re-display time interval, 2 s
uint64_t LCDprevTime = 0;                      // Initialize the last time displayed
bool LCDrefresh = false;                       // Whether to refresh
LiquidCrystal_I2C lcd;                         // Create the LCD object with a default address
char lcd_line[LCDCOLUMNS+1];                   // Note the "+1" to insert an end null!
#endif

///////////////////
// Start of code //
///////////////////
void printMsgSerial() {
#if defined(TRANSMITTER)
// TRANSMITTER
//////////////

  Serial.print("tx: notifyDccMsg: payload(Msg):\n"); 
  Serial.print(" Size: "); Serial.print(payload[0],HEX); Serial.print("\n");
  for(uint8_t i=0; i<payload[0]; i++) {
     Serial.print(" Data[: "); Serial.print(i,HEX); Serial.print("]: ");
     Serial.print(payload[i+1],HEX); Serial.print("\n");
  }

///////////
// TRANSMITTER
#else
// RECEIVER
///////////

  Serial.print("rx: loop: msg["); Serial.print(msgIndexInserted,HEX); Serial.print("]:\n");
  Serial.print(" whatChannel: "); Serial.print(whatChannel,DEC); Serial.print("\n");
  Serial.print(" len: "); Serial.print(msg[msgIndexInserted].Size,HEX); Serial.print("\n");
  for(byte i=0; i<msg[msgIndexInserted].Size; i++) {
     Serial.print(" data["); Serial.print(i,HEX); Serial.print("]: ");
     Serial.print(msg[msgIndexInserted].Data[i],HEX); Serial.print("\n");
  }

///////////
// RECEIVER
#endif
}

// Function, based on the value of forceDefault:
//    - TRUE:   TargetPtr's value and its related EEPROM variables are forced to use defaultValue
//    - FALSE:  extract and use EEPROM data, if previously-set, to set the TargetPtr's value
void checkSetDefaultEE(uint8_t *TargetPtr, 
                       const uint8_t *EEisSetTargetPtr, 
                       const uint8_t *EETargetPtr, 
                       uint8_t defaultValue, 
                       uint8_t forceDefault)
{
   uint8_t isSet, isSet_Save; 
   eeprom_busy_wait();
   isSet = (uint8_t)eeprom_read_byte((const uint8_t *)EEisSetTargetPtr);
   if ((isSet != ISSET) || forceDefault)
   {
      isSet_Save = isSet;
      *TargetPtr = defaultValue; 
      eeprom_busy_wait();

      eeprom_update_byte( (uint8_t *)EEisSetTargetPtr, (const uint8_t)ISSET );
      delay(EEPROMDELAYMS); // Magic delay time to ensure update is complete
      eeprom_busy_wait();
      for (uint8_t i = 0; i < 10; i++)
      {
         isSet = (uint8_t)eeprom_read_byte((const uint8_t *)EEisSetTargetPtr);
         if (isSet != ISSET)
         {
#if defined(DEBUG)
            Serial.print("ISSET error: isSet, ISSET = ");
            Serial.print(isSet);
            Serial.print(", ");
            Serial.print(ISSET);
            Serial.print("\n");
            delay(10);
#endif
            eeprom_update_byte( (uint8_t *)EEisSetTargetPtr, (const uint8_t)ISSET );
            eeprom_busy_wait();
         }
         else break;
      }

      eeprom_update_byte( (uint8_t *)EETargetPtr, defaultValue );
      delay(EEPROMDELAYMS); // Magic delay time to ensure update is complete
      eeprom_busy_wait();

      for (uint8_t i = 0; i < 10; i++)
      {
         *TargetPtr = (uint8_t)eeprom_read_byte((const uint8_t *)EETargetPtr);
         if (*TargetPtr != defaultValue)
         {
#if defined(DEBUG)
            Serial.print("TargetPtr error: *TgtPtr, defaultValue = ");
            Serial.print(*TargetPtr);
            Serial.print(", ");
            Serial.print(defaultValue);
            Serial.print("\n");
            delay(10);
#endif
            eeprom_update_byte( (uint8_t *)EETargetPtr, defaultValue );
            eeprom_busy_wait();
         }
         else break;
      }

#if defined(DEBUG)
   Serial.print("   Reset: *TargetPtr, *EEisSetTargetPtr, *EETargetPtr, ISSET, isSet, forceDefault, defaultValue (Orig isSet): ");
   Serial.print(*TargetPtr);
   Serial.print(" <");
   Serial.print(*EEisSetTargetPtr);
   Serial.print("> <");
   Serial.print(*EETargetPtr);
   Serial.print("> ");
   Serial.print(ISSET);
   Serial.print(" ");
   Serial.print(isSet);
   Serial.print(" ");
   Serial.print(forceDefault);
   Serial.print(" ");
   Serial.print(defaultValue);
   Serial.print(" (");
   Serial.print(isSet_Save);
   Serial.print(")\n");
#endif
   }
   else
   {
      *TargetPtr = (uint8_t)eeprom_read_byte((const uint8_t *)EETargetPtr);
#if defined(DEBUG)
   Serial.print("No Reset: *TargetPtr, *EEisSetTargetPtr, *EETargetPtr, ISSET, isSet, forceDefault, defaultValue: ");
   Serial.print(*TargetPtr);
   Serial.print(" <");
   Serial.print(*EEisSetTargetPtr);
   Serial.print("> <");
   Serial.print(*EETargetPtr);
   Serial.print("> ");
   Serial.print(ISSET);
   Serial.print(" ");
   Serial.print(isSet);
   Serial.print(" ");
   Serial.print(forceDefault);
   Serial.print(" ");
   Serial.print(defaultValue);
   Serial.print("\n");
#endif
   }

   eeprom_busy_wait();

}

#if defined(USE_LCD)
/////////////
// USE_LCD //
void LCD_Banner()
{
  lcd.setCursor(0,0);              // Set initial column, row
  if (LCDwhichBanner==INITIAL) lcd.print(bannerString);   // Banner
  else lcd.print("ProMini Air Info");
  lcd.setCursor(0,1);              // Set next line column, row
  lcd.print("H:1.0 S:1.2/NRF");    // Show state
  LCDprevTime  = micros();     // Set up the previous display time
  LCDrefresh = true;
}

void LCD_Addr_Ch_PL()
{
   lcd.clear();
   lcd.setCursor(0,0); // column, row
   if(printDCC) 
   {
      // Detect long or short address
      tmpuint8 = dccptr->Data[0]&0b11000000;
      if ((tmpuint8==0b11000000) && (dccptr->Data[0]!=0b11111111))
      {
         int TargetAddress_int = ((int)dccptr->Data[0]-192)*256+(int)dccptr->Data[1];
         // snprintf(lcd_line,sizeof(lcd_line),"Msg Ad: %d(%d,%d)",TargetAddress_int,dccptr->Data[0],dccptr->Data[1]);
         snprintf(lcd_line,sizeof(lcd_line),"Msg Ad: %d(L)",TargetAddress_int);
      }
      else
      {
         snprintf(lcd_line,sizeof(lcd_line),"Msg Ad: %d(S)",(int)dccptr->Data[0]);
      }
   }
   else
   {
      if(AirMiniCV29Bit5) 
      {
         int AirMiniAddress_int = ((int)AirMiniCV17-192)*256+(int)AirMiniCV18;
         // snprintf(lcd_line,sizeof(lcd_line),"My Ad: %d(%d,%d)",AirMiniAddress_int,AirMiniCV17,AirMiniCV18);
         snprintf(lcd_line,sizeof(lcd_line),"My Ad: %d(L)",AirMiniAddress_int);
      }
      else
      {
         // snprintf(lcd_line,sizeof(lcd_line),"My Ad(CV1): %d",AirMiniCV1);
         snprintf(lcd_line,sizeof(lcd_line),"My Ad: %d(S)",AirMiniCV1);
      }
   }
    
   lcd.print(lcd_line);
   lcd.setCursor(0,1); // column, row

   if (printDCC) 
   {
      printDCC = 0;
      lcd_line[0] = 'D';
      lcd_line[1] = 'C';
      lcd_line[2] = 'C';
      lcd_line[3] = ':';
      for(uint8_t i = 0; i < dccptr->Size; i++) 
      {
         snprintf(&lcd_line[2*i+4],3,"%02X", dccptr->Data[i]);
      }
   }
   else
   {
      printDCC = 1;
#if defined(TRANSMITTER)
      snprintf(lcd_line,sizeof(lcd_line),"Ch:%d PL:%d", CHANNEL, powerLevel);
#else
      snprintf(lcd_line,sizeof(lcd_line),"Ch:%d Filt:%d", CHANNEL, filterModemData);
#endif
   }

   lcd.print(lcd_line);

   return;
}

void LCD_CVval_Status(uint8_t CVnum, uint8_t CVval)
{
   lcd.clear();

   lcd.setCursor(0,0); // column, row
   switch (CVStatus)
   {
    case ACCEPTED:
       snprintf(lcd_line,sizeof(lcd_line),"Changed:");
    break;
    case IGNORED:
       snprintf(lcd_line,sizeof(lcd_line),"Ignored:");
    break;
    case PENDING:
       snprintf(lcd_line,sizeof(lcd_line),"Pending:");
    break;
   }
   lcd.print(lcd_line);

   lcd.setCursor(0,1); // column, row
   snprintf(lcd_line,sizeof(lcd_line),"CV%d=%d",CVnum,CVval);
   lcd.print(lcd_line);

   LCDprevTime  = micros();
   LCDrefresh = true;

   return;
}

void LCD_Wait_Period_Over(int status)
{
   lcd.clear();
   lcd.setCursor(0,0); // column, row
   if (!status) 
   {
      snprintf(lcd_line,sizeof(lcd_line),"NO valid");
   }
   else
   {
      snprintf(lcd_line,sizeof(lcd_line),"Found valid");
   }
   lcd.print(lcd_line);

   lcd.setCursor(0,1); // column, row 

   snprintf(lcd_line,sizeof(lcd_line),"RF on Ch: %d", CHANNEL);
   lcd.print(lcd_line);
   LCDprevTime  = micros();
   LCDrefresh = true;

   return;
}

// USE_LCD //
/////////////
#endif

#if defined(TRANSMITTER)
// TRANSMITTER
//////////////

extern void notifyDccMsg( DCC_MSG * Msg ) {
    // noInterrupts(); // Turning on/off interrupts does not seem to be needed
    payload[0] = Msg->Size;
    // for(uint8_t i=0; i<payload[0]; i++) payload[i+1] = Msg->Data[i]; // Slow way
    memcpy((void *)&payload[1],(void *)&Msg->Data[0],Msg->Size);
    // interrupts(); // Turning on/off interrupts does not seem to be needed
    radio.write( payload, payload[0]+1, 1 ); // NOACK: Important for broadcast!

    msgIndexInserted = (msgIndexInserted+1) % MAXMSG;
    memcpy((void *)&msg[msgIndexInserted],(void *)Msg,sizeof(DCC_MSG));
    dccptr = &msg[msgIndexInserted];

#ifdef DEBUG
    printMsgSerial();
#endif
} // End of notifyDccMsg

//////////////
// TRANSMITTER
#else
// RECEIVER
///////////

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
} // End of SetupTimer2

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
             memcpy((void *)&msg[msgIndexInserted], (void *)&msgIdle, sizeof(DCC_MSG)); // copy the idle message
             // dccptr = &msg[msgIndexInserted];
          }
          byteIndex = 0; //start msg with byte 0
        }
        break;
      case SEPERATOR:
        timer_val = TIMER_LONG;
        // then advance to next state
        state = SENDBYTE;
        // goto next byte ...
        cbit = 0x80;  // send this bit next time first
        outbyte = msg[msgIndex].Data[byteIndex];
        break;
      case SENDBYTE:
        timer_val = (outbyte & cbit) ? TIMER_SHORT : TIMER_LONG;
        cbit = cbit >> 1;
        if (cbit == 0)  {  // last bit sent, is there a next byte?
          byteIndex++;
          if (byteIndex >= msg[msgIndex].Size)  {
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

} // End of ISR

///////////
// RECEIVER
#endif

void setup() {
#ifdef DEBUG
   Serial.begin(115200);
#endif

   ///////////////////////////////////////////////////////
   // Start: Let's get the slow EEPROM stuff done first //
   ///////////////////////////////////////////////////////

#if defined(DEBUG)
   Serial.print("Initial SET_DEFAULT: "); Serial.print(SET_DEFAULT); Serial.print("\n");
#endif
   SET_DEFAULT = (uint8_t)eeprom_read_byte((const uint8_t *)&EEFirst);
   if (SET_DEFAULT != ISSET) SET_DEFAULT = 1;
   else SET_DEFAULT = 0;
   eeprom_busy_wait();
#if defined(DEBUG)
   Serial.print("Final SET_DEFAULT: "); Serial.print(SET_DEFAULT); Serial.print("\n");
   delay(100);
#endif

  // Just flat out set the powerLevel
  // eeprom_update_byte(&EEpowerLevelDefault, POWERLEVELDEFAULT );
  checkSetDefaultEE(&powerLevel, &EEisSetpowerLevel, &EEpowerLevel, POWERLEVELDEFAULT, 1);  // Force the reset of the power level. This is a conservative approach.


  // Get the CHANNEL # stored in EEPROM and validate it
  // eeprom_update_byte(&EECHANNELDefault, CHANNELDEFAULT );
  checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, CHANNELDEFAULT, SET_DEFAULT);     // Validate the channel, it's possible the EEPROM has bad data
  if(CHANNEL > CHANNELS_MAX) 
      checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, CHANNELDEFAULT, 1);  // Force the EEPROM data to use CHANNEL 10, if the CHANNEL is invalid

  // Get up addressing-related CV's from EEPROM, or if not set set them in EEPROM
  // eeprom_update_byte(&EEAirMiniCV1Default, AIRMINICV1DEFAULT );
  checkSetDefaultEE(&AirMiniCV1,  &EEisSetAirMiniCV1,  &EEAirMiniCV1,    AIRMINICV1DEFAULT, SET_DEFAULT);  // Short address. By default, not using

  // eeprom_update_byte(&EEAirMiniCV17Default, AIRMINICV17DEFAULT );
  checkSetDefaultEE(&AirMiniCV17, &EEisSetAirMiniCV17, &EEAirMiniCV17, AIRMINICV17DEFAULT, SET_DEFAULT);  // High byte to set final address to 9000
  AirMiniCV17tmp = AirMiniCV17;                                                  // Due to the special nature of CV17 paired with CV18

  // eeprom_update_byte(&EEAirMiniCV18Default, AIRMINICV18DEFAULT );
  checkSetDefaultEE(&AirMiniCV18, &EEisSetAirMiniCV18, &EEAirMiniCV18, AIRMINICV18DEFAULT, SET_DEFAULT);  // Low byte to set final address to 9000/9001 for transmitter/receiver

  // eeprom_update_byte(&EEAirMiniCV29Default, AIRMINICV29DEFAULT );
  checkSetDefaultEE(&AirMiniCV29, &EEisSetAirMiniCV29, &EEAirMiniCV29,  AIRMINICV29DEFAULT, SET_DEFAULT);  // Set CV29 so that it will use a long address
  AirMiniCV29Bit5 = AirMiniCV29 & 0b00100000;                                    // Save the bit 5 value of CV29 (0: Short address, 1: Long address)

#if defined(RECEIVER)
  // Set whether to always use modem data on transmit
  // eeprom_update_byte(&EEfilterModemDataDefault, FILTERMODEMDATADEFAULT );
  checkSetDefaultEE(&filterModemData, &EEisSetfilterModemData, &EEfilterModemData, FILTERMODEMDATADEFAULT, SET_DEFAULT);  // Use EEPROM value if it's been set, otherwise set to 0 
#endif

   // Now set to not first time
   eeprom_update_byte( (uint8_t *)&EEFirst, (const uint8_t)ISSET);
   eeprom_busy_wait();

#if defined(USE_LCD)
   LCDprevTime = micros()+LCDTimePeriod;
#endif

   // Set up the input or output pins
#if defined(TRANSMITTER)
// TRANSMITTER
//////////////

   Dcc.pin(EXTINT_NUM, INPUT_PIN, 0); // register External Interrupt # and Input Pin of input source. 
                                      // Important. Pins and interrupt #'s are correlated.

//////////////
// TRANSMITTER
#else
// RECEIVER
///////////

   DDRD |= (1<<OUTPUT_PIN); //  register OUTPUT_PIN for Output source

///////////
// RECEIVER
#endif

   // Start the radio
   radio.begin();
#ifdef DEBUG
   if (radio.isChipConnected()) Serial.print("The chip is connected\n");
   else Serial.print("The chip is NOT connected\n");
#endif

   // Common TX/RX radio handling
   radio.setChannel(CHANNEL); 
   // radio.setDataRate( RF24_250KBPS );
   radio.setDataRate(RF24_1MBPS);
   radio.enableDynamicPayloads();
   radio.enableDynamicAck(); // Added, not sure it's needed on rx side

// Transmitter-specific
#if defined(TRANSMITTER)
   radio.setRetries(0,0); // delay, count
   radio.setPALevel(powerLevel,0); // Experiment with changing
#endif

// Pipe handling (TX/RX specific)
#if defined(TRANSMITTER)
   radio.openWritingPipe(pipe01);
   radio.openReadingPipe(0,pipe00);
   radio.stopListening(); // Experimental. May comment back off
#else
   radio.openWritingPipe(pipe00);
   radio.openReadingPipe(1, pipe01);
   radio.startListening();
#endif
 
// Final timing-sensitive set-ups
#if defined(TRANSMITTER)
// TRANSMITTER
//////////////

   // Call the main DCC Init function to enable the DCC Receiver
   Dcc.init( MAN_ID_DIY, 100,   FLAGS_DCC_ACCESSORY_DECODER, 0 ); 
#ifdef DEBUG
   Serial.println("tx: setup: Dcc package initialized");
#endif

//////////////
// TRANSMITTER
#else
// RECEIVER
///////////

   // Set up the waveform generator timer
   SetupTimer2(); // Set up interrupt Timer 2
#ifdef DEBUG
   Serial.println("rx: setup: timer2 ready");
#endif

///////////
// RECEIVER
#endif

#ifdef DEBUG
#if defined(TRANSMITTER)
   Serial.println("tx: setup: radio ready");
#else
   Serial.println("rx: setup: radio ready");
#endif
#endif

   delay(10); // May need to lengthen

} // End of setup

void loop(){
#if defined(TRANSMITTER)
// TRANSMITTER
//////////////

   Dcc.process(); // The DCC library does it all with the callback notifyDccMsg!

//////////////
// TRANSMITTER
#else
// RECEIVER
///////////

   uint8_t Size;
   if ( radio.available(&whatChannel) ) {
      Size = radio.getDynamicPayloadSize();
      radio.read( &payload, Size );
      noInterrupts();
      msgIndexInserted = (msgIndexInserted+1) % MAXMSG;
      msg[msgIndexInserted].Size = payload[0];
      memcpy((void *)&msg[msgIndexInserted].Data[0],(void *)&payload[1],payload[0]);
      dccptr = &msg[msgIndexInserted];
      interrupts();
#ifdef DEBUG
      if (!print_count) printMsgSerial();
      print_count = (print_count+1) % PRINT_MAX;
#endif

   }

///////////
// RECEIVER
#endif

#if defined(USE_LCD)
   now = micros();             // Grab Clock Value for next time

   if (!lcdInitialized && ((now-LCDprevTime) >= LCDTimePeriod)) {

      lcdInitialized = true;

      // Scan for I2C devices
      Wire.begin(); // Wire communication begin
      byte nDevices = 0;
      byte address;
      byte error;
      for (address = 1; address < 127; address++ )
      {
         // The i2c_scanner uses the return value of
         // the Write.endTransmisstion to see if
         // a device did acknowledge to the address.
         Wire.beginTransmission(address);
         error = Wire.endTransmission();
     
         if (error == 0)
         {
           nDevices++;
           LCDAddress = address;
         }
      }

      if (nDevices == 1)
      {
         LCDFound = true;
         LCDrefresh = true;
         lcd.init(LCDAddress,LCDCOLUMNS,LCDROWS);    // Initialize the LCD
         lcd.backlight();                            // Backlight it
         LCDwhichBanner = INITIAL;
         LCDprevTime = now+LCDTimePeriod;
      }
      else 
      {
         LCDFound = false;
         LCDrefresh = false;
      }

   }

   if(LCDrefresh && ((now-LCDprevTime) >= LCDTimePeriod)) {
       if (LCDwhichBanner==NONE) LCD_Addr_Ch_PL();           // Update the display of address, chanel #, and power level
       else if(!initialWait) {
          if (LCDwhichBanner==INITIAL){
             LCD_Banner();
             LCDwhichBanner = INFO;
          }
          else {
             LCD_Banner();
             LCDwhichBanner = NONE;
          }
       }
       LCDprevTime = now;              // Slowly... at 1 sec intervals
       LCDrefresh = true;
   }
#endif

} // end of loop

// End of file
