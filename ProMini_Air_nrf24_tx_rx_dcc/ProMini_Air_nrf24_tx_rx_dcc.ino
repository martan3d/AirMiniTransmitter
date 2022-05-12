// ProMini_Air_nrf24_tx_rx_dcc.ino - transmit/receive DCC date 30 May 2021
//////////////////////////////////////////////////////////////////////////////////////////
// For compilation output messages. Most are simply informational to ensure the correct 
// #define's have been used. 
// Some of the error checking WILL halt compilation with an ERROR.
#define xstr(x) str(x)
#define str(x) #x
// #define DO_PRAGMA(x) _Pragma(str(x))
//////////////////////////////////////////////////////////////////////////////////////////

#undef TRANSMITTER
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

#define USE_OPS_MODE

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
#include <EEPROM.h>

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
uint8_t initialWait = 1;                     // Initial wait status for receiving valid DCC
#endif

enum {INITIAL, INFO, NONE} LCDwhichBanner = INITIAL;

uint16_t CVnum;                              // CV numbers consume 10 bits
uint8_t CVval;                               // CV values consume only 8 bits


#define ISSET 0b10101010

uint8_t SET_DEFAULT = 1;
uint8_t EEMEM EEFirst;  // Store the first time

// Channel-related
#if defined(TRANSMITTER)
#define LNADEFAULT 0          // 
#else
#define LNADEFAULT 1          // 
#endif

uint8_t LNA = LNADEFAULT;  // 
uint8_t EEMEM EEisSetLNA;      // Stored RF channel is set
uint8_t EEMEM EELNA;           // Stored RF channel #
// uint8_t EEMEM EELNADefault; // Stored RF channel #

// Channel-related
#define CHANNELDEFAULT 10          // 
#define CHANNELS_MAX 127           // 
uint8_t CHANNEL = CHANNELDEFAULT;  // 
uint8_t EEMEM EEisSetCHANNEL;      // Stored RF channel is set
uint8_t EEMEM EECHANNEL;           // Stored RF channel #
// uint8_t EEMEM EECHANNELDefault; // Stored RF channel #

// Power-related
#define POWERLEVELDEFAULT RF24_PA_MIN
#define POWERLEVEL_MAX 3            // 
uint8_t powerLevel = POWERLEVELDEFAULT; // The modem power level (>=0 and <=10). Communicated to spi.c
uint8_t EEMEM EEisSetpowerLevel;        // Stored DC output power level is set
uint8_t EEMEM EEpowerLevel;             // Stored DC output power level 
// uint8_t EEMEM EEpowerLevelDefault;   // Stored DC output power level 

#if defined(RECEIVER)
//{ RECEIVER
// Filtering-related
#define FILTERMODEMDATADEFAULT 0
uint8_t filterModemData = FILTERMODEMDATADEFAULT; // Set the logical for whether to always use modem data.
uint8_t EEMEM EEisSetfilterModemData;             // Stored filter modem data set flag
uint8_t EEMEM EEfilterModemData;                  // Stored filter modem data in ms
// uint8_t  EEMEM EEfilterModemDataDefault;       // Stored filter modem data in ms
//} RECEIVER
#endif

// CV-Related
#define AIRMINICV1DEFAULT 3

#if ! defined(AIRMINICV17DEFAULT)
#define AIRMINICV17DEFAULT 230
#endif
#pragma message "Info: Default CV17 is " xstr(AIRMINICV17DEFAULT)

#if ! defined(AIRMINICV18DEFAULT)
//{
#if defined(TRANSMITTER)
#define AIRMINICV18DEFAULT 72
#else
#define AIRMINICV18DEFAULT 73
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

// DCC_MSG type defined in NmraDcc.h
volatile DCC_MSG *dccptrIn;
volatile DCC_MSG *dccptrTmp;
volatile DCC_MSG *dccptrOut;
volatile bool printIn = true;

bool newMsg = false;
uint8_t modemCVResetCount=0;
DCC_MSG dccptrAirMiniCVReset[sizeof(DCC_MSG)] = {0, 0, {0,0,0,0,0,0}};
DCC_MSG dccptrNULL[sizeof(DCC_MSG)] = {0, 0, {0,0,0,0,0,0}};
uint8_t tmpuint8 = 0;
uint8_t countPtr = 1;

#if defined(TRANSMITTER)
//{ TRANSMITTER

// Actual input pin. No conversion to PD3! Dcc.init does this.
//#define INPUT_PIN 2
//#define EXTINT_NUM 0

#define INPUT_PIN 3
#define EXTINT_NUM 1

//} TRANSMITTER
#else
//{ RECEIVER

uint8_t whatChannel;
#if defined(DEBUG)
#define PRINT_MAX 129
int print_count = 0;
#endif

// use digital pins 6 and 5 for DCC out
// use digital pin D3
#define OUTPUT_PIN PD3
#define OUTPUT_PIN2 PD4

#define OUTPUT_HIGH   PORTD = (PORTD |  (1<<OUTPUT_PIN)) & ~(1<<OUTPUT_PIN2)
#define OUTPUT_LOW    PORTD = (PORTD & ~(1<<OUTPUT_PIN)) |  (1<<OUTPUT_PIN2)
#define SET_OUTPUTPIN DDRD  = (DDRD  |  (1<<OUTPUT_PIN)) |  (1<<OUTPUT_PIN2)

//} RECEIVER
#endif

//Settings for both TX/RX
RF24 radio(9,10); // CE,CSN

#if defined(TRANSMITTER)
//{ TRANSMITTER

NmraDcc Dcc;


//} TRANSMITTER
#else
//{ RECEIVER

//Timer frequency is 2MHz for ( /8 prescale from 16MHz )
#define TIMER_SHORT 0x8D  // 58usec pulse length
#define TIMER_LONG  0x1B  // 116usec pulse length


// definitions for state machine
// uint8_t last_timer = TIMER_SHORT; // store last timer value
// uint8_t timer_val = TIMER_LONG; // The timer value
volatile uint8_t timer_val = TIMER_SHORT; // The timer value
volatile uint8_t every_second_isr = 0;  // pulse up or down

volatile enum {PREAMBLE, SEPERATOR, SENDBYTE} state = PREAMBLE;
volatile byte preamble_count = 16;
volatile byte outbyte = 0;
volatile byte cbit = 0x80;
volatile int byteIndex = 0;

//} RECEIVER
#endif

#define MAXMSG 32

volatile DCC_MSG msg[MAXMSG] = {
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   // idle msg
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}},   //
  { 3, 16, {0xFF, 0, 0xFF, 0, 0, 0}}    //
};

const DCC_MSG msgIdle =
   {3, 16, {0xFF, 0, 0xFF, 0, 0, 0}};   // idle msg

volatile uint8_t msgIndexOut = 0;
volatile uint8_t msgIndexIn = 0; // runs from 0 to MAXMSG-1

#if defined(TRANSMITTER)
//{ TRANSMITTER

// const byte slaveAddress[5] = {'R','x','A','A','A'};
const uint64_t pipe00 = 0xE8E8F0F0A0ULL;
const uint64_t pipe01 = 0xE8E8F0F0A1ULL;
const uint64_t pipe02 = 0xE8E8F0F0A2ULL;  
const uint64_t pipe03 = 0xE8E8F0F0A3ULL;
const uint64_t pipe04 = 0xE8E8F0F0A4ULL;
const uint64_t pipe05 = 0xE8E8F0F0A5ULL;
const uint64_t pipe06 = 0xE8E8F0F0A6ULL;

//} TRANSMITTER
#else
//{ RECEIVER

// const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
const uint64_t pipe00 = 0xE8E8F0F0A0ULL;
const uint64_t pipe01 = 0xE8E8F0F0A1ULL;
const uint64_t pipe02 = 0xA2ULL;  
const uint64_t pipe03 = 0xA3ULL;
const uint64_t pipe04 = 0xA4ULL;
const uint64_t pipe05 = 0xA5ULL;
const uint64_t pipe06 = 0xA6ULL;

//} RECEIVER
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


#if defined(USE_OPS_MODE)
uint8_t restartModemFlag = 0;                  // Initial setting for calling startModem under some circumstances
#endif

#if defined(RECEIVER)
//{ RECEIVER
#define INITIALWAITPERIODSECDEFAULT 1
uint64_t timeOfValidDCC;             // Time stamp of the last valid DCC packet
uint64_t startInitialWaitTime;       // The start of the initial wait time. Will be set in initialization
uint64_t endInitialWaitTime;         // The end of the initial wait time. Will be set in initialization
uint8_t InitialWaitPeriodSEC = INITIALWAITPERIODSECDEFAULT;    // Wait period
uint8_t searchChannelIndex = 0;      // Initialial channel search order index
uint8_t  EEMEM EEisSetInitialWaitPeriodSEC;  // Stored AirMini decoder configuration variable
uint8_t  EEMEM EEInitialWaitPeriodSEC;       // Stored AirMini decoder configuration variable
//} RECEIVER
#endif

///////////////////
// Start of code //
///////////////////
#if defined(DEBUG)
//{ DEBUG
void printMsgSerial() {
#if defined(TRANSMITTER)
//{ TRANSMITTER

  Serial.print("tx: notifyDccMsg: payload(Msg):\n"); 
  Serial.print(" Size: "); Serial.print(payload[0],HEX); Serial.print("\n");
  for(uint8_t i=0; i<payload[0]; i++) {
     Serial.print(" Data[: "); Serial.print(i,HEX); Serial.print("]: ");
     Serial.print(payload[i+1],HEX); Serial.print("\n");
  }

//} TRANSMITTER
#else
//{ RECEIVER

  Serial.print("rx: loop: msg["); Serial.print(msgIndexIn,HEX); Serial.print("]:\n");
  Serial.print(" whatChannel: "); Serial.print(whatChannel,DEC); Serial.print("\n");
  Serial.print(" len: "); Serial.print(msg[msgIndexIn].Size,HEX); Serial.print("\n");
  for(byte i=0; i<msg[msgIndexIn].Size; i++) {
     Serial.print(" data["); Serial.print(i,HEX); Serial.print("]: ");
     Serial.print(msg[msgIndexIn].Data[i],HEX); Serial.print("\n");
  }

//} RECEIVER
#endif
}
//} DEBUG
#endif

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
  lcd.print("H:1.0 S:1.6/NRF");    // Show state
  LCDprevTime  = micros();     // Set up the previous display time
  LCDrefresh = true;
}

void LCD_Addr_Ch_PL()
{
   dccptrTmp = dccptrIn;
   if (!printIn) dccptrTmp = dccptrOut;

   lcd.clear();
   lcd.setCursor(0,0); // column, row
   if(printDCC) 
   {
      // Detect long or short address
      tmpuint8 = dccptrTmp->Data[0]&0b11000000;
      if ((tmpuint8==0b11000000) && (dccptrTmp->Data[0]!=0b11111111))
      {
         int TargetAddress_int = ((int)dccptrTmp->Data[0]-192)*256+(int)dccptrTmp->Data[1];
         // snprintf(lcd_line,sizeof(lcd_line),"Msg Ad: %d(%d,%d)",TargetAddress_int,dccptrTmp->Data[0],dccptrTmp->Data[1]);
         snprintf(lcd_line,sizeof(lcd_line),"Msg Ad: %d(L)",TargetAddress_int);
      }
      else
      {
         snprintf(lcd_line,sizeof(lcd_line),"Msg Ad: %d(S)",(int)dccptrTmp->Data[0]);
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
      snprintf(lcd_line,sizeof(lcd_line),"                ");
      printDCC = 0;
      lcd_line[0] = 'P';
      lcd_line[1] = 'M';
      lcd_line[2] = 'A';
      if (printIn) {
         lcd_line[3] = '<';
         printIn = false;
      }
      else {
         lcd_line[3] = '>';
         printIn = true;
      }
      for(uint8_t i = 0; i < dccptrTmp->Size; i++) 
      {
         snprintf(&lcd_line[2*i+4],3,"%02X", dccptrTmp->Data[i]);
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
//{ TRANSMITTER

extern void notifyDccMsg( DCC_MSG * Msg ) {
    // noInterrupts(); // Turning on/off interrupts does not seem to be needed
    payload[0] = Msg->Size;
    // for(uint8_t i=0; i<payload[0]; i++) payload[i+1] = Msg->Data[i]; // Slow way
    memcpy((void *)&payload[1],(void *)&Msg->Data[0],Msg->Size);
    // interrupts(); // Turning on/off interrupts does not seem to be needed
    radio.write( payload, payload[0]+1, 1 ); // NOACK: Important for broadcast!

    msgIndexIn = (msgIndexIn+1) % MAXMSG;
    memcpy((void *)&msg[msgIndexIn],(void *)Msg,sizeof(DCC_MSG));
    dccptrIn = &msg[msgIndexIn];
    msgIndexOut = msgIndexIn;
    dccptrOut = dccptrIn;
    newMsg = true;

#if defined(DEBUG)
    printMsgSerial();
#endif
} // End of notifyDccMsg

//} TRANSMITTER
#else
//{ RECEIVER

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
  // byte latency;

  // for every second interupt just toggle signal
  if (every_second_isr)  {

    // PORTD = B01000000;  //use this instead of digitalWrite(6, 1); digitalWrite(5, 0);
    // PORTD |= (1<<OUTPUT_PIN);  // OUTPUT_PIN High
    OUTPUT_HIGH;
    every_second_isr = 0;
    // set timer to last value
    // latency = TCNT2;
    // TCNT2 = latency + last_timer;

  }  else  {  // != every second interrupt, advance bit or state

    // PORTD &= ~(1<<OUTPUT_PIN);  // OUTPUT_PIN Low
    OUTPUT_LOW;
    every_second_isr = 1;

    switch (state)  {
      case PREAMBLE:
        timer_val = TIMER_SHORT;
        preamble_count--;
        if (preamble_count == 0)  {  // advance to next state
          state = SEPERATOR;
          // get next message
          if (msgIndexOut != msgIndexIn) {
             msgIndexOut = (msgIndexOut+1) % MAXMSG;
          }
          else {// If no new message, send an idle message in the updated msgIndexIn slot
             msgIndexIn = (msgIndexIn+1) % MAXMSG;
             msgIndexOut = msgIndexIn;
             memcpy((void *)&msg[msgIndexOut], (void *)&msgIdle, sizeof(DCC_MSG)); // copy the idle message
          }
          dccptrOut = &msg[msgIndexOut];
          byteIndex = 0; //start msg with byte 0
        }
        break;
      case SEPERATOR:
        timer_val = TIMER_LONG;
        // then advance to next state
        state = SENDBYTE;
        // goto next byte ...
        cbit = 0x80;  // send this bit next time first
        outbyte = msg[msgIndexOut].Data[byteIndex];
        break;
      case SENDBYTE:
        timer_val = (outbyte & cbit) ? TIMER_SHORT : TIMER_LONG;
        cbit = cbit >> 1;
        if (cbit == 0)  {  // last bit sent, is there a next byte?
          byteIndex++;
          if (byteIndex >= msg[msgIndexOut].Size)  {
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
    // latency = TCNT2;
    // TCNT2 = latency + timer_val;
    // last_timer = timer_val;


  } // end of else ! every_seocnd_isr

  // Set up output timer. It is only reset every second ISR.
  TCNT2 += timer_val;

} // End of ISR

//} RECEIVER
#endif

#if defined(USE_OPS_MODE)
void reboot() {
   cli();                    // Ensure that when setup() is called, interrupts are OFF
   asm volatile ("  jmp 0"); // "Dirty" method because it simply restarts the SW, and does NOT reset the HW 
}

void eepromClear() {
   for (int i = 0 ; i < (int)(EEPROM.length()) ; i++) {
      EEPROM.write(i, 0xFF);
   }
}

void ops_mode()
{
         newMsg = false;
         /////////////////////////////////////////////
         // Special processing for AirMini OPS mode //
         /////////////////////////////////////////////
         if(((dccptrIn->Data[0]==AirMiniCV17) && (dccptrIn->Data[1]== AirMiniCV18) &&  AirMiniCV29Bit5) ||
            ((dccptrIn->Data[0]==AirMiniCV1)                               && !AirMiniCV29Bit5) )
         {
            // According the NMRA standards, two identical packets should be received
            // before modifying CV values. 
            countPtr = 1;
            if (AirMiniCV29Bit5) countPtr = 2;
            tmpuint8 = dccptrIn->Data[countPtr]&(0b11111100); // The last two bits are part of the CV address used below and we don't care right now.
                                                            // Do NOT increment countPtr because we aren't finished withe dccptrIn->Data[countPtr] yet;
                                                            // we need its two low bytes for the upper two bytes of the CV address below!
            if(tmpuint8==0b11101100)                          // Determine if the bit pattern is for modifying CV's with the last two bits don't care
            {
               if(modemCVResetCount==0)                   // Processing for identifying first or second valid call
               {
                  modemCVResetCount++;                                 // Update the CV reset counter
                  memcpy((void *)&dccptrAirMiniCVReset,(void *)dccptrIn,sizeof(DCC_MSG)); // Save the dcc packet for comparison
               }
               else 
               {
                  if(!memcmp((void *)&dccptrAirMiniCVReset,(void *)dccptrIn,sizeof(DCC_MSG)))  // If they don't compare, break out
                  {
                     restartModemFlag = 0;             // Initialize whether the modem will be restarted
                     tmpuint8 = dccptrIn->Data[countPtr++]&(0b00000011); // zero out the first 6 bits of dccptrIn->Data[countPtr], we want to use the last two bits
                     CVnum = (uint16_t)tmpuint8;     // cast the result to a 2 uint8_t CVnum
                     CVnum <<= 8;                    // now move these two bit over 8 places, now that it's two bytes
                     uint16_t tmpuint16 = (uint16_t)dccptrIn->Data[countPtr++]; // cast dccptrIn->Data[countPtr] to 2 bytes
                     CVnum |= tmpuint16;             // set the last 8 bits with dccptrIn->Data[countPtr]
                     CVnum++;                        // NMRA Std of plus one, good grief, to set the final CV number
                     CVval = dccptrIn->Data[countPtr++]; // Set CVval to dccptrIn->Data[countPtr], one uint8_t only!
                     CVStatus = ACCEPTED;            // Set the default CV status

	              switch(CVnum)
                     {
                        case  255:  // Set the channel number and reset related EEPROM values. Modest error checking. Verified this feature works
                           if(CVval <= CHANNELS_MAX)           // Check for good values
                           {
                              checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, (uint8_t)CVval, 1);  
                              restartModemFlag = 1;
                           }
                           else                      // Ignore bad values
                              CVStatus = IGNORED;
                        break;
                        case  254:  // Set the RF power level and reset related EEPROM values. Verified this feature works.
                           if(CVval<=POWERLEVEL_MAX) 
                           {
                              checkSetDefaultEE(&powerLevel, &EEisSetpowerLevel, &EEpowerLevel, (uint8_t)CVval, 1); // Set powerLevel and reset EEPROM values. Ignore bad values
                              restartModemFlag = 1;
                           }
                           else
                              CVStatus = IGNORED;
                        break;
                        case  253:  // Set the LNA (0=off, !0=on)
                           if(CVval) LNA = 1;
                           else LNA = 0;
                           checkSetDefaultEE(&LNA, &EEisSetLNA, &EELNA, (uint8_t)LNA, 1); // Set powerLevel and reset EEPROM values. Ignore bad values
                           restartModemFlag = 1;
                        break;
                        case 29:    // Set the Configuration CV and reset related EEPROM values. Verified this feature works.
                           checkSetDefaultEE(&AirMiniCV29, &EEisSetAirMiniCV29, &EEAirMiniCV29, (uint8_t)CVval, 1); 
                           AirMiniCV29Bit5 = AirMiniCV29 & 0b00100000; // Save the bit 5 value of CV29 (0: Short address, 1: Long address)
                        break;
                        case 18:    // Set the Long Address Low Byte CV and reset related EEPROM values. Verified this feature works.
                                    // See NMRA S-9.2.1 Footnote 8.
                           checkSetDefaultEE(&AirMiniCV17, &EEisSetAirMiniCV17, &EEAirMiniCV17, (uint8_t)AirMiniCV17tmp, 1); 
                           checkSetDefaultEE(&AirMiniCV18, &EEisSetAirMiniCV18, &EEAirMiniCV18, (uint8_t)CVval, 1); 
                        break;
                        case 17:    // Set the Long Address High Byte CV and save values after validation (do NOT write to AirMini's CV17 or EEPROM yet!).
                           if((0b11000000<=CVval) && (CVval<=0b11100111))  // NMRA standard 9.2.2, Paragraphs 129-135, footnote 8
                           {
                              AirMiniCV17tmp = CVval;    // Do not take effect until CV18 is written! NMRA Standard 9.2.1, footnote 8.
                              CVStatus = PENDING;
                           }
                           else
                              CVStatus = IGNORED;
                        break;
                        case  8:  // Full EEPROM Reset and reboot!
                           if (CVval==8) 
                           {
#if defined(USE_LCD)
                              if (LCDFound) 
                              {
                                 lcd.clear();
                                 lcd.setCursor(0,0); // column, row
                                 snprintf(lcd_line,sizeof(lcd_line),"Keep Power ON!");
                                 lcd.print(lcd_line);
                                 lcd.setCursor(0,1); // column, row
                                 snprintf(lcd_line,sizeof(lcd_line),"Factory Reset...");
                                 lcd.print(lcd_line);
                              }
#endif
                              eepromClear();
                              reboot(); // No need for sei, you're starting over...
                           }
                           else {
                              CVStatus = IGNORED;
                           }
                        break;
                        case 1:     // Set the Short Address CV and reset related EEPROM values after validation. Verified this feature works.
                           if((0<CVval) && (CVval<128))  // CV1 cannot be outside this range. Some decoders limit 0<CVval<100
                           {
                              checkSetDefaultEE(&AirMiniCV1, &EEisSetAirMiniCV1, &EEAirMiniCV1, (uint8_t)CVval, 1); 
                           }
                           else
                              CVStatus = IGNORED;
                        break;
                        default:
                           CVStatus = IGNORED;
                        break;
                     } // end of switch(CVnum) 

#if defined(USE_LCD)
                     if (LCDFound) LCD_CVval_Status(CVnum,CVval);
#endif

                     if(restartModemFlag)
                     {
                        radio.setChannel(CHANNEL); 
                        radio.setPALevel(powerLevel,LNA); 
                     }

                  } // end of if(!memcmp...

                  modemCVResetCount=0;                                     // Reset the CV reset counter
                  memcpy((void *)&dccptrAirMiniCVReset,(void *)&dccptrNULL,sizeof(DCC_MSG)); // Reset the dcc packet for comparison

               } // end of else (modemCVResetCount ...

            } // end of if(tmpuint8 ...
            else  // Not in OPS mode
            {
               modemCVResetCount=0;           // Reset this counter if we didn't get a "hit" on CV changing
            } // End of not in OPS mode

         } // end of if((dccptrIn->Data[0] ...
         ///////////////////////////////////////////////////
         // End ofSpecial processing for AirMini OPS mode //
         ///////////////////////////////////////////////////

} // end of ops_mode

#endif

#if defined(RECEIVER)
//{ RECEIVER
void channel_search() {

   // If we received a valid DCC signal during the intial wait period, stop waiting and proceed normally
   if (timeOfValidDCC > startInitialWaitTime) 
   {
      initialWait = 0; 
#if defined(USE_LCD)
      if (LCDFound) LCD_Wait_Period_Over(1);
#endif
   }
   else  // Othewise, continue to wait
   {
      now = micros();
      // If it's too long, then reset the modem to the next channel
      if (now > endInitialWaitTime) 
      {
#if defined(USE_LCD)
         if (LCDFound) LCD_Wait_Period_Over(0);
#endif
         if (++searchChannelIndex <= CHANNELS_MAX+1)
         // Keep searching...
         {
            // Update the seach channel and endInitialWaitTime
            CHANNEL = searchChannelIndex-1;
            // Re-initialize the start of the wait time
            startInitialWaitTime = now;      
            // Re-initialize the end of the wait time
            endInitialWaitTime = 
                startInitialWaitTime
              + (uint64_t)InitialWaitPeriodSEC * SEC; 
         }
         else 
         // Last resort
         {
            initialWait = 0;
            CHANNEL=CHANNELDEFAULT;    // Reset to the last resort channel
         }

         // Set the radio to the new channel
         radio.setChannel(CHANNEL); 
         radio.setPALevel(powerLevel,LNA); 

      } // end of wait time over
   } // end of continue to wait
} // end of channel_search
//} RECEIVER
#endif

void setup() {
#if defined(DEBUG)
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

  // Get the LNA # stored in EEPROM and validate it
  // eeprom_update_byte(&EELNADefault, LNADEFAULT );
  checkSetDefaultEE(&LNA, &EEisSetLNA, &EELNA, LNADEFAULT, SET_DEFAULT);     // Validate the channel, it's possible the EEPROM has bad data

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
//{ RECEIVER
  // Set whether to always use modem data on transmit
  // eeprom_update_byte(&EEfilterModemDataDefault, FILTERMODEMDATADEFAULT );
  checkSetDefaultEE(&filterModemData, &EEisSetfilterModemData, &EEfilterModemData, FILTERMODEMDATADEFAULT, SET_DEFAULT);  // Use EEPROM value if it's been set, otherwise set to 0 
   // eeprom_update_byte(&EEInitialWaitPeriodSECDefault, INITIALWAITPERIODSECDEFAULT );
   checkSetDefaultEE(&InitialWaitPeriodSEC, &EEisSetInitialWaitPeriodSEC, &EEInitialWaitPeriodSEC,  (uint8_t)INITIALWAITPERIODSECDEFAULT, SET_DEFAULT);  // Wait time in sec
//} RECEIVER
#endif

#if defined(USE_OPS_MODE)
   // Get up addressing-related CV's from EEPROM, or if not set set them in EEPROM
   // eeprom_update_byte(&EEAirMiniCV1Default, AIRMINICV1DEFAULT );
   checkSetDefaultEE(&AirMiniCV1,  &EEisSetAirMiniCV1,  &EEAirMiniCV1,    (uint8_t)AIRMINICV1DEFAULT, SET_DEFAULT);  // Short address. By default, not using

   // eeprom_update_byte(&EEAirMiniCV17Default, AIRMINICV17DEFAULT );
   checkSetDefaultEE(&AirMiniCV17, &EEisSetAirMiniCV17, &EEAirMiniCV17, (uint8_t)AIRMINICV17DEFAULT, SET_DEFAULT);  // High uint8_t to set final address to 9000
   AirMiniCV17tmp = AirMiniCV17;                                                  // Due to the special nature of CV17 paired with CV18

   // eeprom_update_byte(&EEAirMiniCV18Default, AIRMINICV18DEFAULT );
   checkSetDefaultEE(&AirMiniCV18, &EEisSetAirMiniCV18, &EEAirMiniCV18, (uint8_t)AIRMINICV18DEFAULT, SET_DEFAULT);  // Low uint8_t to set final address to 9000/9001 for transmitter/receiver

   // eeprom_update_byte(&EEAirMiniCV29Default, AIRMINICV29DEFAULT );
   checkSetDefaultEE(&AirMiniCV29, &EEisSetAirMiniCV29, &EEAirMiniCV29,  (uint8_t)AIRMINICV29DEFAULT, SET_DEFAULT);  // Set CV29 so that it will use a long address
   AirMiniCV29Bit5 = AirMiniCV29 & 0b00100000;                                    // Save the bit 5 value of CV29 (0: Short address, 1: Long address)

#endif

   // Now set to not first time
   eeprom_update_byte( (uint8_t *)&EEFirst, (const uint8_t)ISSET);
   eeprom_busy_wait();

#if defined(USE_LCD)
   LCDprevTime = micros()+LCDTimePeriod;
#endif

   // Set up the input or output pins
#if defined(TRANSMITTER)
//{ TRANSMITTER

   Dcc.pin(EXTINT_NUM, INPUT_PIN, 0); // register External Interrupt # and Input Pin of input source. 
                                      // Important. Pins and interrupt #'s are correlated.

//} TRANSMITTER
#else
//{ RECEIVER

   SET_OUTPUTPIN; //  register OUTPUT_PIN for Output source


//} RECEIVER
#endif

   // Start the radio
   radio.begin();
#if defined(DEBUG)
   if (radio.isChipConnected()) Serial.print("The chip is connected\n");
   else Serial.print("The chip is NOT connected\n");
#endif

   // Common TX/RX radio handling
   radio.setChannel(CHANNEL); 
   radio.setDataRate( RF24_250KBPS ); // Lower rate works, better range
   // radio.setDataRate(RF24_1MBPS);
   radio.enableDynamicPayloads();
   radio.enableDynamicAck(); // Added, not sure it's needed on rx side

   dccptrIn  = &msg[msgIndexIn]; // Initialize dccptrIn
   dccptrOut = &msg[msgIndexOut]; // Initialize dccptrOut

// Transmitter-specific
#if defined(TRANSMITTER)
   radio.setRetries(0,0); // delay, count - Important for tranmitter broadcast!
#endif

   radio.setPALevel(powerLevel,LNA); // Set the power level and LNA

// Pipe handling (TX/RX specific)
#if defined(TRANSMITTER)
//{ TRANSMITTER
   radio.openWritingPipe(pipe01);
   radio.openReadingPipe(0,pipe00);
   radio.stopListening(); // Experimental. May comment back off
//} TRANSMITTER
#else
//{ RECEIVER
   radio.openWritingPipe(pipe00);
   radio.openReadingPipe(1, pipe01);
   radio.startListening();
//} RECEIVER
#endif
 
// Final timing-sensitive set-ups
#if defined(TRANSMITTER)
//{ TRANSMITTER

   // Call the main DCC Init function to enable the DCC Receiver
   Dcc.init( MAN_ID_DIY, 100,   FLAGS_DCC_ACCESSORY_DECODER, 0 ); 
#if defined(DEBUG)
   Serial.println("tx: setup: Dcc package initialized");
#endif

//} TRANSMITTER
#else
//{ RECEIVER

   // Set up the waveform generator timer
   SetupTimer2(); // Set up interrupt Timer 2
#if defined(DEBUG)
   Serial.println("rx: setup: timer2 ready");
#endif
   newMsg = false;
   timeOfValidDCC = micros();

   initialWait = 1;
   startInitialWaitTime = timeOfValidDCC;      // Initialize the start of the wait time (never is not a good value)
   endInitialWaitTime = 
      startInitialWaitTime
    + (uint64_t)InitialWaitPeriodSEC * SEC; // Initialize the end of the wait time

//} RECEIVER
#endif

#if defined(DEBUG)
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
//{ TRANSMITTER

   Dcc.process(); // The DCC library does it all with the callback notifyDccMsg!

//} TRANSMITTER
#else
//{ RECEIVER

   uint8_t Size;
   if ( radio.available(&whatChannel) ) {
      Size = radio.getDynamicPayloadSize();
      radio.read( &payload, Size );
      noInterrupts();
      msgIndexIn = (msgIndexIn+1) % MAXMSG;
      msg[msgIndexIn].Size = payload[0];
      memcpy((void *)&msg[msgIndexIn].Data[0],(void *)&payload[1],payload[0]);
      dccptrIn = &msg[msgIndexIn];
      newMsg = true;
      timeOfValidDCC = micros();
      interrupts();
#if defined(DEBUG)
      if (!print_count) printMsgSerial();
      print_count = (print_count+1) % PRINT_MAX;
#endif

   }

//} RECEIVER
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

#if defined(USE_OPS_MODE)
   if (newMsg) ops_mode();
#endif

#if defined(RECEIVER)
   if (initialWait) channel_search();
#endif

} // end of loop

// End of file
