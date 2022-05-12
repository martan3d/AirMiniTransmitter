/*
AirMiniSketchTransmitter.ino 

Created: Dec  7 12:15:25 EST 2019

Copyright (c) 2019-2021, Martin Sant and Darrell Lamm
All rights reserved.

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

#define DEBUG_LOCAL

#include <config.h>
#include <EEPROM.h>
#include <dcc.h>
#include <spi.h>
#include <uart.h>
#include <schedule.h>
#include <servotimer.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <string.h>

#if defined(TRANSMITTER)
#undef RECEIVER
#define DCCLibrary
#define USE_LCD
#elif defined(RECEIVER)
#undef DCCLibrary
#define USE_LCD
#else
#error "Error: Neither TRANSMITTER or RECEIVER is defined"
#endif

#if defined(USE_LCD)
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#endif

#define OUTPUT_ENABLE 5 // Output Enable
#define DCC_DIAG1 6     // Diagnostic Pin #2

#if defined(DCCLibrary)
///////////////////
///////////////////
///////////////////
#include <DCCLibrary.h>
// Externs
extern bool (*GetNextMessage)(void); // For DCCLibrary

// For DCCLibrary
#if defined(TRANSMITTER)
#define DCC_PIN    2     // Arduino pin for DCC out to modem 
#else
#define DCC_PIN    4     // Arduino pin for DCC out, not currently used
#endif

#define MAXMSG 32       // The size of the ring buffer. Per Martin's new code

// Implement a ring buffer
volatile Message msg[MAXMSG] = {      // -> to DCCLibrary.c
    { 3, 16, { 0xFF, 0, 0xFF, 0, 0, 0}},
    { 3, 16, { 0xFF, 0, 0xFF, 0, 0, 0}},
    { 0, 0,  { 0,    0, 0,    0, 0, 0}}
   };

// Private msg sent to DCCLibrary.c ISR
volatile Message msgExtracted[2] = {    // -> to DCCLibrary.c
    { 3, 16, { 0xFF, 0, 0xFF, 0, 0, 0}}, // Will be overwritten in NextMessage
    { 3, 16, { 0xFF, 0, 0xFF, 0, 0, 0}}, // The idle packet we will send in DCCLibrary.c when conditions permit/necessitate
   };

// Idle message
const Message msgIdle = 
   { 3, 16, { 0xFF, 0, 0xFF, 0, 0, 0}};   // idle msg

extern volatile uint8_t msgExtractedIndex; // From ISR in DCCLibrary

volatile uint8_t lastMessageInserted  = 1;
volatile uint8_t lastMessageExtracted = 0;

///////////////////
///////////////////
///////////////////
#endif


// Times
// #if defined(TRANSMITTER)
// #define INITIALDELAYMS       1000   // Initial processor start-up delay in ms
// #else
// #define INITIALDELAYMS       1000   // Initial processor start-up delay in ms
// #endif

#define EEPROMDELAYMS       100   // Delay after eeprom write in ms
#define MILLISEC        1000ULL   //    1 msec. Units: us
#define QUARTERSEC    250000ULL   // 0.25  sec. Units: us
#define SEC          1000000ULL   // 1.00  sec. Units: us
#define BACKGROUNDTIME  8000ULL   //    8 msec. Units: us

// CC1101 codes
#define RXMODE  0x34             // C1101 modem RX mode
#define RX      0x34
#define TX      0x35
#define SIDLE   0x36

// Setting up double pass for OPS mode
#if defined(TRANSMITTER)
#define DOUBLE_PASS 1            // Do a double pass on CV setting 
#else
#define DOUBLE_PASS 1            // Do a double pass on CV setting 
#endif

// DEFAULT defines
extern uint8_t channels_max;     // From spi.c
extern uint8_t channels_na_max;  // From spi.c

#if defined(TRANSMITTER)
#define POWERLEVELDEFAULT 8
#else
#define POWERLEVELDEFAULT 6
#endif

#define DCLEVEL_INDEFAULT 1
#define TURNMODEMON_INDEFAULT 0
#define IDLEPERIODMSDEFAULT 128
#define FILTERMODEMDATADEFAULT 0
#define AIRMINICV1DEFAULT 3

#if ! defined(AIRMINICV17DEFAULT)
#define AIRMINICV17DEFAULT 230
#endif
#pragma message "Info: Default CV17 is " xstr(AIRMINICV17DEFAULT)

#if ! defined(AIRMINICV18DEFAULT)
//{
#if defined(TRANSMITTER)
#define AIRMINICV18DEFAULT 172
#else
#define AIRMINICV18DEFAULT 173
#endif
//}
#endif
#pragma message "Info: Default CV18 is " xstr(AIRMINICV18DEFAULT)

#define AIRMINICV29DEFAULT 32

#if defined(RECEIVER)
#define INITIALWAITPERIODSECDEFAULT 1
#else
//{
#if ! defined(AUTOIDLEOFFDEFAULT)
#define AUTOIDLEOFFDEFAULT 0
#endif
//}
#endif
#pragma message "Info: Default AUTOIDLEOFFDEFAULT is " xstr(AUTOIDLEOFFDEFAULT)


// Declarations
uint64_t now;
uint64_t then;

volatile DCC_MSG *dccptr;

uint8_t modemCVResetCount=0;
uint8_t sendbuffer[sizeof(DCC_MSG)];
uint8_t dccptrAirMiniCVReset[sizeof(DCC_MSG)];
uint8_t dccptrNULL[sizeof(DCC_MSG)];
uint8_t dccptrRepeatCount = 0;
uint8_t dccptrRepeatCountMax = 2;
uint8_t msgReplaced = 0;
uint8_t tmpuint8 = 0;
uint8_t do_not_filter = 0;
uint8_t countPtr = 1;

#if defined(TRANSMITTER)
uint8_t MODE = TX;                           // Mode is now a variable. Don't have the courage to change 
#else
uint8_t MODE = RX;                           // Mode is now a variable. Don't have the courage to change 
                                             // it in SW since you can't change it back in SW w/o a reset and 
                                             // using EEPROM data
#endif

uint8_t startModemFlag = 0;                  // Initial setting for calling startModem under some circumstances
uint8_t filterModemData;                     // Set the logical for whether to always use modem data. Initialized elsewhere
volatile uint8_t useModemData = 1;           // Initial setting for use-of-modem-data state
uint64_t idlePeriod = 128;                   // 128 msec, changed to variable that might be changed by SW
uint8_t idlePeriodms = 0;                    // 0 msec, changed to variable that might be changed by SW
uint64_t lastIdleTime = 0;
uint64_t tooLong=2ULL*SEC;           // 1 sec, changed to variable that might be changed by SW
uint64_t sleepTime = 0;              // 0 sec, changed to variable that might be changed by SW
uint64_t timeOfValidDCC;             // Time stamp of the last valid DCC packet
uint64_t inactiveStartTime;          // Time stamp if when modem data is ignored
uint16_t maxTransitionCount;              // Maximum number of bad transitions to tolerate before ignoring modem data
uint8_t maxTransitionCountLowByte=100;       // High uint8_t of maxTransitionCount
uint8_t maxTransitionCountHighByte=0;        // Low uint8_t of maxTransitionCount
#if defined(RECEIVER)
uint64_t startInitialWaitTime;       // The start of the initial wait time. Will be set in initialization
uint64_t endInitialWaitTime;         // The end of the initial wait time. Will be set in initialization
uint8_t InitialWaitPeriodSEC;                // Wait period
uint8_t searchChannelIndex = 0;              // Initialial channel search order index
#else
uint8_t AutoIdleOff;                         // Automatic Idle off. Will be intialized later
#endif

#if defined(TRANSMITTER)
uint8_t initialWait = 0;                     // Initial wait status for receiving valid DCC
#else
uint8_t initialWait = 1;                     // Initial wait status for receiving valid DCC
#endif
enum {INITIAL, INFO, NONE} whichBanner = INITIAL;
uint8_t regionNum=0;
#if defined(NAEU_900MHz)
//{
#pragma message "Info: using European 869MHz/North American 915MHz frequency-dependent channels"
#if defined(RECEIVER)
uint8_t searchChannels[18] = {0,17,16,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; // Channel search order
#endif
#if defined(USE_LCD)
const char *bannerString = "ProMini Air NA/E";
const char *regionString[] = {"N","E"}; // Region code: N=North America, E=Europe, W=Worldwide
bool lcdInitialized = false;
#endif
//}
#else
//{

#if defined(EU_434MHz)
//{
#pragma message "Info: using European 434MHz frequency-dependent channels"
#if defined(RECEIVER)
uint8_t searchChannels[8] = {0,1,2,3,4,5,6,7}; // Channel search order
#endif
#if defined(USE_LCD)
const char *bannerString = "ProMini Air EU";
const char *regionString[] = {"E"}; // Region code: N=North America, E=Europe, W=Worldwide
#endif
//}
#else
//{

#if defined(NAEU_2p4GHz)
//{
#pragma message "Info: using Worldwide 2.4GHz frequency-dependent channels"
#if defined(RECEIVER)
uint8_t searchChannels[8] = {0,1,2,3,4,5,6,7}; // Channel search order
#endif
#if defined(USE_LCD)
const char *bannerString = "ProMini Air WW";
const char *regionString[] = {"W"}; // Region code: N=North America, E=Europe, W=Worldwide
#endif
//}
#endif

//}
#endif

//}
#endif



// Changing with CV's
uint16_t CVnum;                              // CV numbers consume 10 bits
uint8_t CVval;                               // CV values consume only 8 bits
uint8_t CHANNEL;                             // Airwire Channel for both TX/RX, may change in SW. Do NOT initialize
volatile uint8_t turnModemOnOff;             // Do NOT intialize, in EEPROM
uint8_t turnModemOnOff_in;                   // Non-volatile version
volatile uint8_t dcLevel;                    // The output level (HIGH or LOW) output if modem data is invalid
extern uint8_t powerLevel;                   // The modem power level (>=0 and <=10). Communicated to spi.c
uint8_t dcLevel_in;                          // Non-volatile version

uint8_t AirMiniCV1;                          // The AirMini's address, HIGH uint8_t

uint8_t AirMiniCV17;                         // The AirMini's address, HIGH uint8_t
uint8_t AirMiniCV17tmp;                      // The AirMini's address, HIGH uint8_t, temporary value until CV18 is reassigned in ops mode

uint8_t AirMiniCV18;                         // The AirMini's address, LOW uint8_t

uint8_t AirMiniCV29;                         // The AirMini's address, HIGH uint8_t
uint8_t AirMiniCV29Bit5;                     // The value of AirMiniCV29, bit 5
uint8_t printDCC = 1;                        // Global flag for LCD for DCC msg display

/////////////////////
// Start: EEPROM data
/////////////////////

#define ISSET 0b10101010

uint8_t SET_DEFAULT = 1;
uint8_t EEFirst = 0;  // Store the first time

// EEPROM data for persistence after turn-off of the AirMini
uint8_t  EEisSetCHANNEL = 1;               // Stored RF channel is set
uint8_t  EECHANNEL = 2;                    // Stored RF channel #
// uint8_t  EECHANNELDefault;             // Stored RF channel #

uint8_t  EEisSetturnModemOnOff = 3;        // Stored modem turn on/off is set
uint8_t  EEturnModemOnOff = 4;             // Stored modem turn on/off
// uint8_t  EEturnModemOnOffDefault;      // Stored modem turn on/off

uint8_t  EEisSetdcLevel = 5;               // Stored DC output level is set
uint8_t  EEdcLevel = 6;                    // Stored DC output level if modem turned off
// uint8_t  EEdcLevelDefault;             // Stored DC output level if modem turned off

uint8_t  EEisSetpowerLevel = 7;            // Stored DC output power level is set
uint8_t  EEpowerLevel = 8;                 // Stored DC output power level 
// uint8_t  EEpowerLevelDefault;          // Stored DC output power level 

uint8_t  EEisSetidlePeriodms = 9;          // Stored idlePeriodms set flag
uint8_t  EEidlePeriodms = 10;               // Stored idlePeriod in ms
// uint8_t  EEidlePeriodmsDefault;        // Stored idlePeriod in ms

uint8_t  EEisSetfilterModemData = 11;       // Stored idlePeriodms set flag
uint8_t  EEfilterModemData = 12;            // Stored idlePeriod in ms
// uint8_t  EEfilterModemDataDefault;     // Stored idlePeriod in ms

uint8_t  EEisSetAirMiniCV1 = 13;            // Stored AirMini decoder short address is set
uint8_t  EEAirMiniCV1 = 14;                 // Stored AirMini decoder short address
// uint8_t  EEAirMiniCV1Default;          // Stored AirMini decoder short address

uint8_t  EEisSetAirMiniCV17 = 15;           // Stored AirMini decoder high uint8_t address is set
uint8_t  EEAirMiniCV17 = 16;                // Stored AirMini decoder high uint8_t address
// uint8_t  EEAirMiniCV17Default;

uint8_t  EEisSetAirMiniCV18 = 17;           // Stored AirMini decoder low uint8_t address is set
uint8_t  EEAirMiniCV18 = 18;                // Stored AirMini decoder low uint8_t address
// uint8_t  EEAirMiniCV18Default;         // Stored AirMini decoder low uint8_t address

uint8_t  EEisSetAirMiniCV29 = 19;           // Stored AirMini decoder configuration variable is set
uint8_t  EEAirMiniCV29 = 20;                // Stored AirMini decoder configuration variable
// uint8_t  EEAirMiniCV29Default;         // Stored AirMini decoder configuration variable

#if defined(RECEIVER)
uint8_t  EEisSetInitialWaitPeriodSEC = 21;  // Stored AirMini decoder configuration variable
uint8_t  EEInitialWaitPeriodSEC = 22;       // Stored AirMini decoder configuration variable
// uint8_t  EEInitialWaitPeriodSECDefault;// Stored AirMini decoder configuration variable
#else
uint8_t  EEisSetAutoIdleOff = 23;  // Stored AirMini decoder configuration variable
uint8_t  EEAutoIdleOff = 24;       // Stored AirMini decoder configuration variable
// uint8_t  EEAutoIdleOffDefault;// Stored AirMini decoder configuration variable
#endif

///////////////////
// End: EEPROM data
///////////////////

enum {ACCEPTED, IGNORED, PENDING} CVStatus = ACCEPTED;

#if defined(USE_LCD)
uint8_t LCDAddress;                     // The I2C address of the LCD
bool LCDFound = false;               // Whether a valid lcd was found
#define LCDCOLUMNS 16                // Number of LCD columns
#define LCDROWS 2                    // Number of LCD rows 
uint64_t LCDTimePeriod=2ULL*SEC;// Set up the LCD re-display time interval, 2 s
uint64_t LCDprevTime = 0;       // Initialize the last time displayed
bool LCDrefresh = false;             // Whether to refresh
LiquidCrystal_I2C lcd;               // Create the LCD object with a default address
char lcd_line[LCDCOLUMNS+1];         // Note the "+1" to insert an end null!
#endif

///////////////////
// Start of code //
///////////////////

/******************************************************************************/
#if defined(DCCLibrary)
///////////////////
///////////////////
///////////////////
bool NextMessage(void){  // Sets for DCCLibrary.c's access to msg
    
#if ! defined(DONTTURNOFFINTERRUPTS)
   cli(); // turn off interrupts
#endif
   bool retval = false;

   // Set the last message extracted from the ring buffer so DCCLibrary.c can use it
   if(lastMessageExtracted != lastMessageInserted)  // Update the extract pointer because we have not caught up the the inserted pointer
   {
      lastMessageExtracted = (lastMessageExtracted+1) % MAXMSG;
      retval = true;
   }

   memcpy((void *)&msgExtracted[0], (void *)&msg[lastMessageExtracted], sizeof(Message));    // Extract the message into private msg
#if ! defined(DONTTURNOFFINTERRUPTS)
   sei(); // turn on interrupts
#endif

   return retval;
}
///////////////////
///////////////////
///////////////////
#endif

// Function to combine High and Low bytes into 16 uint8_t variable
uint16_t combineHighLow(uint8_t High, uint8_t Low)
{
   uint16_t tmp16 = (uint16_t)High;
   tmp16 <<=8; 
   uint16_t tmpLow16 = (uint16_t)Low;
   tmp16 |= tmpLow16;
   return (tmp16);
}

void reboot() {
   cli();                    // Ensure that when setup() is called, interrupts are OFF
   asm volatile ("  jmp 0"); // "Dirty" method because it simply restarts the SW, and does NOT reset the HW 
}

void eepromClear() {
   for (int i = 0 ; i < (int)(EEPROM.length()) ; i++) {
      EEPROM.write(i, 0);
   }
}

void checkSetDefault(uint8_t *TargetPtr, const uint8_t *EEisSetTargetPtr, const uint8_t *EETargetPtr, uint8_t defaultValue, uint8_t forceDefault)
{

   Serial.print("   Check: *TargetPtr, *EEisSetTargetPtr, *EETargetPtr, ISSET, isSet, defaultValue: ");
   Serial.print(*TargetPtr);
   Serial.print(" <");
   Serial.print(*EEisSetTargetPtr);
   Serial.print("> <");
   Serial.print(*EETargetPtr);
   Serial.print("> ");
   Serial.print(defaultValue);
   Serial.print("\n");

}

// Function, based on the value of forceDefault:
//    - TRUE:   TargetPtr's value and its related EEPROM variables are forced to use defaultValue
//    - FALSE:  extract and use EEPROM data, if previously-set, to set the TargetPtr's value
void checkSetDefaultEE(uint8_t *TargetPtr, const uint8_t *EEisSetTargetPtr, const uint8_t *EETargetPtr, uint8_t defaultValue, uint8_t forceDefault)
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
void LCD_Banner()
{
   lcd.setCursor(0,0);              // Set initial column, row
   if (whichBanner==INITIAL) lcd.print(bannerString);   // Banner
   else lcd.print("ProMini Air Info");
   lcd.setCursor(0,1);              // Set next line column, row
#if defined(TWENTY_SEVEN_MHZ)
//{
   lcd.print("H:1.1 S:2.3/27MH");   // Show state
//}
#else
//{
#if defined(TWENTY_SIX_MHZ)
   lcd.print("H:1.1 S:2.3/26MH");   // Show state
#else
//{
#error "Undefined crystal frequency"
//}
#endif
//}
#endif
   LCDprevTime  = micros();     // Set up the previous display time
   LCDrefresh = true;
}

void LCD_Addr_Ch_PL()
{
   lcd.clear();
   lcd.setCursor(0,0); // column, row
   /*
   uint16_t AirMiniAddress = (uint16_t)(AirMiniCV17&0b00111111);
   AirMiniAddress <<= 8;
   AirMiniAddress |= AirMiniCV17;
   int AirMiniAddress_int = (int)AirMiniAddress;
   // snprintf(lcd_line,sizeof(lcd_line),"Addr: %d",AirMiniAddress_int);
   */
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
      lcd_line[0] = 'P';
      lcd_line[1] = 'M';
      lcd_line[2] = 'A';
      lcd_line[3] = ':';
      for(uint8_t i = 0; i < dccptr->Size; i++) 
      {
         snprintf(&lcd_line[2*i+4],3,"%02X", dccptr->Data[i]);
      }
   }
   else
   {
      printDCC = 1;
#if defined(NAEU_900MHz)
      if (CHANNEL <= channels_na_max)
         regionNum=0;
      else
         regionNum=1;
#endif

#if defined(TRANSMITTER)
      snprintf(lcd_line,sizeof(lcd_line),"Ch:%d(%s) PL:%d", CHANNEL, regionString[regionNum], powerLevel);
#else
      snprintf(lcd_line,sizeof(lcd_line),"Ch:%d(%s) Filt:%d", CHANNEL, regionString[regionNum], filterModemData);
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

#if defined(NAEU_900MHz)
   if (CHANNEL <= channels_na_max)
     regionNum=0;
   else
     regionNum=1;
#endif

   snprintf(lcd_line,sizeof(lcd_line),"RF on Ch: %d(%s)",CHANNEL, regionString[regionNum]);
   lcd.print(lcd_line);
   LCDprevTime  = micros();
   LCDrefresh = true;

   return;
}
#endif

void setup() 
{

   // delay(INITIALDELAYMS);

#if defined(DEBUG) || defined(DEBUG_LOCAL)
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

   // Get the CHANNEL # stored in EEPROM and validate it
   // eeprom_update_byte(&EECHANNELDefault, CHANNELDEFAULT );
   checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, (uint8_t)CHANNELDEFAULT, SET_DEFAULT);      // Validate the channel, it's possible the EEPROM has bad data
   if(CHANNEL > channels_max) 
      checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, (uint8_t)CHANNELDEFAULT, 1);  // Force the EEPROM data to use CHANNEL 0, if the CHANNEL is invalid
  
   // Just flat out set the powerLevel
   // eeprom_update_byte(&EEpowerLevelDefault, POWERLEVELDEFAULT );
   checkSetDefaultEE(&powerLevel, &EEisSetpowerLevel, &EEpowerLevel, (uint8_t)POWERLEVELDEFAULT, 1);  // Force the reset of the power level. This is a conservative approach.

   // Set the alternate DC output level to HIGH or LOW (i.e., bad CC1101 data)
   // The level of this output can be used by some decoders. The default is HIGH.
   // eeprom_update_byte(&EEdcLevelDefault, DCLEVEL_INDEFAULT );
   checkSetDefaultEE(&dcLevel_in, &EEisSetdcLevel, &EEdcLevel, (uint8_t)DCLEVEL_INDEFAULT, SET_DEFAULT);       // Use EEPROM value if it's been set, otherwise set to 1 and set EEPROM values
   dcLevel = (volatile uint8_t)dcLevel_in;                                  // Since dcLevel is volatile we need a proxy uint8_t 

   // Turn the modem OFF/ON option. For use if bad modem data is detected in RX mode
   // eeprom_update_byte(&EEturnModemOnOffDefault, TURNMODEMON_INDEFAULT );
   checkSetDefaultEE(&turnModemOnOff_in, &EEisSetturnModemOnOff, &EEturnModemOnOff, (uint8_t)TURNMODEMON_INDEFAULT, SET_DEFAULT);  // Use EEPROM value if it's been set, otherwise set to 1 and set EEPROM values
   turnModemOnOff = (volatile uint8_t)turnModemOnOff_in;                                    // Needed to use a proxy variable since turnModemOnOff is volatile uint8_t

   // Set the DCC time-out period for sending IDLE packets. Used along with duplicate DCC packet detection for inserting IDLE packets
   // eeprom_update_byte(&EEidlePeriodmsDefault, IDLEPERIODMSDEFAULT );
   checkSetDefaultEE(&idlePeriodms, &EEisSetidlePeriodms, &EEidlePeriodms, (uint8_t)IDLEPERIODMSDEFAULT, SET_DEFAULT);  // Use EEPROM value if it's been set, otherwise set to 0 ms
   idlePeriod = (uint64_t)idlePeriodms * MILLISEC;                                 // Convert to time counts

   // Set whether to always use modem data on transmit
   // eeprom_update_byte(&EEfilterModemDataDefault, FILTERMODEMDATADEFAULT );
   checkSetDefaultEE(&filterModemData, &EEisSetfilterModemData, &EEfilterModemData, (uint8_t)FILTERMODEMDATADEFAULT, SET_DEFAULT);  // Use EEPROM value if it's been set, otherwise set to 0 

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

#if defined(RECEIVER)
   // eeprom_update_byte(&EEInitialWaitPeriodSECDefault, INITIALWAITPERIODSECDEFAULT );
   checkSetDefaultEE(&InitialWaitPeriodSEC, &EEisSetInitialWaitPeriodSEC, &EEInitialWaitPeriodSEC,  (uint8_t)INITIALWAITPERIODSECDEFAULT, SET_DEFAULT);  // Wait time in sec
#else
   // eeprom_update_byte(&EEAutoIdleOffDefault, AUTOIDLEOFFDEFAULT );
   checkSetDefaultEE(&AutoIdleOff, &EEisSetAutoIdleOff, &EEAutoIdleOff,  (uint8_t)AUTOIDLEOFFDEFAULT, SET_DEFAULT);  // Set AutoIdleOff
#endif

   // Now set to not first time
   eeprom_update_byte( (uint8_t *)&EEFirst, (const uint8_t)ISSET);
   eeprom_busy_wait();

   /////////////////////////////////////////////////////
   // End: Let's get the slow EEPROM stuff done first //
   /////////////////////////////////////////////////////


   /////////////////////////////////
   // Initialization of variables //
   /////////////////////////////////

   startModemFlag = 0;                          // Initialize the start-modem flag
   useModemData = 1;                            // Initialize use-of-modem-data state
   DCCuseModemData(useModemData,dcLevel);       // Tell the DCC code if you are or are not using modem data

   maxTransitionCount = combineHighLow(maxTransitionCountHighByte,maxTransitionCountLowByte);
   modemCVResetCount = 0;

   memset(dccptrNULL,0,sizeof(dccptrNULL));                      // Create a null dccptr for CV setting
   memset(dccptrAirMiniCVReset,0,sizeof(dccptrAirMiniCVReset));  // Initialize the reset dccptr for CV setting


   ///////////////////////////////////////////////
   // Set up the hardware and related variables //
   ///////////////////////////////////////////////

   pinMode(OUTPUT_ENABLE,OUTPUT);             // 
   digitalWrite(OUTPUT_ENABLE,1);             // Output Enable

   pinMode(DCC_DIAG1,OUTPUT);                 //
   digitalWrite(DCC_DIAG1,0);                 // Will use this for diagnostics

#if defined(DCCLibrary)
   ////////////////////
   // For DCCLibrary //
   ////////////////////
   GetNextMessage = &NextMessage;             // assign a proper function to GetNextMessage that actually does something

   //Set the pin for DCC to "output" and set up DCC timers
   SetupDCC(DCC_PIN);   

   // Experimental: Initially clear TASK1
   clearScheduledTask(TASK1);

   /////////////////////
   /////////////////////
   /////////////////////
 #endif

   initServoTimer();                           // Master Timer plus servo timer

   // Set up slow-time variables
   then = micros();                            // Grab Current Clock value for the loop below

#if defined(USE_LCD)
   LCDprevTime = micros()+LCDTimePeriod;
#endif

   timeOfValidDCC = micros();                      // Initialize the valid DCC data time
#if defined(RECEIVER)
   initialWait = 1;
   startInitialWaitTime = timeOfValidDCC;      // Initialize the start of the wait time (never is not a good value)
   endInitialWaitTime = 
      startInitialWaitTime
    + (uint64_t)InitialWaitPeriodSEC * SEC; // Initialize the end of the wait time
#else
   initialWait = 0;
#endif
   inactiveStartTime = micros() + BACKGROUNDTIME;  // Initialize the modem idle time into the future

   // Start the coms with the modem
   initializeSPI();                            // Initialize the SPI interface to the radio
   delay(10);                                  // Wait a bit for the SPI
   dccInit();                                  // Enable DCC transmit/receive
   startModem(CHANNEL, MODE);                  // Start radio on this Channel

   sei();                                      // enable interrupts

}
// End of setup


void loop()
{

   /* Check High Priority Tasks First */


   switch( masterSchedule() )
   {
      case TASK0:                      // Highest Priority Task goes here
                                       // We don't have one right now so this is just a placeholder
         clearScheduledTask(TASK0);
         break;

      case TASK1:                      // Just pick a priority for the DCC packet, TASK1 will do 

         dccptr = getDCC();       // we are here, so a packet has been assembled, get a pointer to our DCC data

         if (memcmp((void *)sendbuffer,(void *)dccptr,sizeof(DCC_MSG))) dccptrRepeatCount=0;  // If they don't match, reset the repeat count
         else dccptrRepeatCount++;                                            // If they do match, increment the repeat count

         // decodeDCCPacket((DCC_MSG*) dccptr);      // Send debug data
         memcpy((void *)sendbuffer,(void *)dccptr,sizeof(DCC_MSG));

#if defined(TRANSMITTER)
//{

#if ! defined(DONTTURNOFFINTERRUPTS)
         cli(); // Turn off interrupts
#endif

#if defined(DCCLibrary)
         lastMessageInserted = (lastMessageInserted+1) % MAXMSG;  // Set the last message inserted into the ring buffer 
#endif
         // Detect long or short address
         tmpuint8 = dccptr->Data[0]&0b11000000;
         if ((tmpuint8==0b11000000) && (dccptr->Data[0]!=0b11111111)) countPtr = 2;
         else countPtr = 1;
         // if either a short or long address message is to change the CV, DO NOT FILTER IT OUT!
         tmpuint8 = dccptr->Data[countPtr]&0b11111100;
         do_not_filter = (tmpuint8==0b11101100) ? 1 : 0;

         // Logic to pass through packet or send an IDLE packet to keep Airwire keep-alive
         // if((dccptrRepeatCount < dccptrRepeatCountMax) || (((uint64_t)micros() - lastIdleTime) < idlePeriod) || AutoIdleOff) 
         if(((dccptrRepeatCount < dccptrRepeatCountMax) && (((uint64_t)micros() - lastIdleTime) < idlePeriod)) || AutoIdleOff || do_not_filter)
         {                     
#if defined(DCCLibrary)
            memcpy((void *)&msg[lastMessageInserted],(void *)dccptr,sizeof(DCC_MSG)); // Dangerous, fast copy
#endif
            msgReplaced = 0;
         }
         else                                       // Send out an idle packet (for keep-alive!)
         {
            dccptrRepeatCount = 0;
            lastIdleTime = micros();
            msgReplaced = 1;
#if defined(DCCLibrary)
            memcpy((void *)&msg[lastMessageInserted],(void *)&msgIdle,sizeof(DCC_MSG)); // Dangerous, fast copy
#endif
         }
#if ! defined(DONTTURNOFFINTERRUPTS)
         sei(); // Turn interrupts back on
#endif

#if defined(DEBUG_LOCAL)
#if defined(DCCLibrary)
         if (msgExtractedIndex || msgReplaced) 
         {
            Serial.print("Replaced message with msgIdle: ISR/Local: ");
            Serial.print(msgExtractedIndex); 
            Serial.print("/");
            Serial.print(msgReplaced); 
            Serial.print("\n");
         }
#else
         if (msgReplaced) 
         {
            Serial.print("Replaced message with msgIdle\n");
         }
#endif
#endif

//} TRANSMITTER
#endif

#if defined(RECEIVER)
         if(!useModemData) // If not using modem data, ensure that the output is set to a DC level after coming back from the ISR
         {
            if(dcLevel) OUTPUT_HIGH;                // HIGH
            else OUTPUT_LOW;                        // LOW
         }
#endif

         /////////////////////////////////////////////
         // Special processing for AirMini OPS mode //
         /////////////////////////////////////////////
         if(((dccptr->Data[0]==AirMiniCV17) && (dccptr->Data[1]== AirMiniCV18) &&  AirMiniCV29Bit5) ||
            ((dccptr->Data[0]==AirMiniCV1)                               && !AirMiniCV29Bit5) )
         {
            // According the NMRA standards, two identical packets should be received
            // before modifying CV values. This feature now works (i.e., DOUBLE_PASS=1(=true)).
            countPtr = 1;
            if (AirMiniCV29Bit5) countPtr = 2;
            tmpuint8 = dccptr->Data[countPtr]&(0b11111100); // The last two bits are part of the CV address used below and we don't care right now.
                                                            // Do NOT increment countPtr because we aren't finished withe dccptr->Data[countPtr] yet;
                                                            // we need its two low bytes for the upper two bytes of the CV address below!
            if(tmpuint8==0b11101100)                          // Determine if the bit pattern is for modifying CV's with the last two bits don't care
            {
               if(modemCVResetCount==0 && DOUBLE_PASS)                   // Processing for identifying first or second valid call
               {
                  modemCVResetCount++;                                 // Update the CV reset counter
                  memcpy((void *)dccptrAirMiniCVReset,(void *)dccptr,sizeof(DCC_MSG)); // Save the dcc packet for comparison
               }
               else 
               {
                  if(!memcmp((void *)dccptrAirMiniCVReset,(void *)dccptr,sizeof(DCC_MSG)) || !DOUBLE_PASS)  // If they don't compare, break out
                  {
                     startModemFlag = 0;             // Initialize whether the modem will be restarted
                     tmpuint8 = dccptr->Data[countPtr++]&(0b00000011); // zero out the first 6 bits of dccptr->Data[countPtr], we want to use the last two bits
                     CVnum = (uint16_t)tmpuint8;     // cast the result to a 2 uint8_t CVnum
                     CVnum <<= 8;                    // now move these two bit over 8 places, now that it's two bytes
                     uint16_t tmpuint16 = (uint16_t)dccptr->Data[countPtr++]; // cast dccptr->Data[countPtr] to 2 bytes
                     CVnum |= tmpuint16;             // set the last 8 bits with dccptr->Data[countPtr]
                     CVnum++;                        // NMRA Std of plus one, good grief, to set the final CV number
                     CVval = dccptr->Data[countPtr++]; // Set CVval to dccptr->Data[countPtr], one uint8_t only!
                     CVStatus = ACCEPTED;            // Set the default CV status

	              switch(CVnum)
                     {
                        case  255:  // Set the channel number and reset related EEPROM values. Modest error checking. Verified this feature works
                           if(CVval <= channels_max)           // Check for good values
                           {
                              checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, (uint8_t)CVval, 1);  
                              startModemFlag = 1;
                           }
                           else                      // Ignore bad values
                              CVStatus = IGNORED;
                        break;
                        case  254:  // Set the RF power level and reset related EEPROM values. Verified this feature works.
                           if(CVval<=10) 
                           {
                              checkSetDefaultEE(&powerLevel, &EEisSetpowerLevel, &EEpowerLevel, (uint8_t)CVval, 1); // Set powerLevel and reset EEPROM values. Ignore bad values
                              startModemFlag = 1;
                           }
                           else
                              CVStatus = IGNORED;
                        break;
                        case  253:  // Turn off/on the modem for bad packet intervals and reset related EEPROM values. Verified this feature works
                           checkSetDefaultEE(&turnModemOnOff_in, &EEisSetturnModemOnOff, &EEturnModemOnOff, (uint8_t)CVval, 1); // Set turnModemOnOff and reset EEPROM values
                           turnModemOnOff = (volatile uint8_t)turnModemOnOff_in; // Assign for volatile
                        break;
                        case  252:  // Set the tooLong (in quarter second intervals) and reset related EEPROM values. 
                           tooLong = (uint64_t)CVval * QUARTERSEC;
                        break;
                        case  251:  // Set the sleepTime (in quarter second intervals) and reset related EEPROM values. Verified this feature works.
                           sleepTime = (uint64_t)CVval * QUARTERSEC;
                        break;
                        case  250:  // Set the low uint8_t for transition counts
                           maxTransitionCountLowByte = CVval;
                           maxTransitionCount = combineHighLow(maxTransitionCountHighByte,maxTransitionCountLowByte);
                        break;
                        case  249:  // Set the high uint8_t for transition counts
                           maxTransitionCountHighByte = CVval;
                           maxTransitionCount = combineHighLow(maxTransitionCountHighByte,maxTransitionCountLowByte);
                        break;
                        case  248:  // Set the DC output level and reset related EEPROM values. Verified this feature works.
                           checkSetDefaultEE(&dcLevel_in, &EEisSetdcLevel, &EEdcLevel, (uint8_t)CVval, 1); // Set dcLevel and reset EEPROM values
                           dcLevel = (volatile uint8_t)dcLevel_in;
                        break;
                        case  247:  // Set the idle period (in ms) and reset related EEPROM values. Verified it works.
                           checkSetDefaultEE(&idlePeriodms, &EEisSetidlePeriodms, &EEidlePeriodms, (uint8_t)CVval, 1); // Set idlePeriodms and reset EEPROM values (in ms!)
                           idlePeriod = (uint64_t)idlePeriodms * MILLISEC; // Convert to cycles
                        break;
                        case  246:  // Set whether to always use modem data
                           if (CVval) CVval = 1; // Non-zero reset to 1
                           checkSetDefaultEE(&filterModemData, &EEisSetfilterModemData, &EEfilterModemData, (uint8_t)CVval, 1); // Set filterModemData and reset EEPROM values
                        break;
#if defined(RECEIVER)
                        case  245:  // Set the wait period in 1 second intervals - Nothing can be done with this until reset
                           if(CVval <= 60)
                              checkSetDefaultEE(&InitialWaitPeriodSEC, &EEisSetInitialWaitPeriodSEC, &EEInitialWaitPeriodSEC,  (uint8_t)CVval, 1);  // Wait time in sec
                           else
                              CVStatus = IGNORED;
                        break;
#endif
#if defined(TRANSMITTER)
                        case  244:  // Turn off automatic IDLE insertion
                           checkSetDefaultEE(&AutoIdleOff, &EEisSetAutoIdleOff, &EEAutoIdleOff,  (uint8_t)CVval, 1); 
                        break;
#endif
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

                     if(startModemFlag)
                     {
                        strobeSPI(SIDLE);         // Stop the modem
                        dccInit();                 // Reset the DCC state machine, which also resets transitionCount
                        startModem(CHANNEL, MODE); // Restart on possible-new Airwire Channel and mode (or power level)
                     }


                  } // end of if(!memcmp...

                  modemCVResetCount=0;                                     // Reset the CV reset counter
                  memcpy((void *)dccptrAirMiniCVReset,(void *)dccptrNULL,sizeof(DCC_MSG)); // Reset the dcc packet for comparison

               } // end of else (modemCVResetCount ...

            } // end of if(tmpuint8 ...
            else  // Not in OPS mode
            {
               modemCVResetCount=0;           // Reset this counter if we didn't get a "hit" on CV changing
            } // End of not in OPS mode

         } // end of if((dccptr->Data[0] ...
         ///////////////////////////////////////////////////
         // End ofSpecial processing for AirMini OPS mode //
         ///////////////////////////////////////////////////

         timeOfValidDCC = micros();  // Grab Current Clock value for the checking below
         clearScheduledTask(TASK1);      // all done, come back next time we are needed
  
      break; // TASK1 break
   } // End of switch( masterSchedule() )

   /**** After checking highest priority stuff, check for the timed tasks ****/

   now = micros();

#if defined(RECEIVER)
   if(!useModemData)        // If not using modem data, ensure the output is set to DC after coming back from the ISR
   {
      if(dcLevel) OUTPUT_HIGH;           // HIGH
      else OUTPUT_LOW;                   // LOW
   }
#endif
   
   if( (now - then) > BACKGROUNDTIME )            // Check for Time Scheduled Tasks
   {                                   // A priority Schedule could be implemented in here if needed
      then = micros();             // Grab Clock Value for next time

#if defined(USE_LCD)
      if (!lcdInitialized && ((then-LCDprevTime) >= LCDTimePeriod)) 
      {

         lcdInitialized = true;

         // Scan for I2C devices
         Wire.begin(); // Wire communication begin
         uint8_t nDevices = 0;
         uint8_t address;
         uint8_t error;
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
            whichBanner = INITIAL;
            LCDprevTime = then+LCDTimePeriod;
         }
         else 
         {
            LCDFound = false;
            LCDrefresh = false;
         }

      }

      if(LCDrefresh && ((then-LCDprevTime) >= LCDTimePeriod)) 
      {
          if (whichBanner==NONE) LCD_Addr_Ch_PL();           // Update the display of address, chanel #, and power level
          else if(!initialWait) {
             if (whichBanner==INITIAL){
                LCD_Banner();
                whichBanner = INFO;
             }
             else {
                LCD_Banner();
                whichBanner = NONE;
             }
          }
          LCDprevTime = then;              // Slowly... at 1 sec intervals
          LCDrefresh = true;
      }
#endif

#if defined(TRANSMITTER)
      strobeSPI(MODE);         // keep the radio awake in MODE 
#else
//{
      if(useModemData)
      {
         strobeSPI(MODE);         // keep the radio awake in MODE 

         // If the DCC data collection appears hung up, put the modem to sleep (if we want to)
         // and just ignore its data for a while...
         // The modem's output will "stick" to either LOW or HIGH (it's random!) when turned off, 
         // causing the amplifier to go LOW or HIGH. The Airwire forces output high when no RF 
         // (or really keep-alive) is received. We will simply ignore the modem data when
         // we think it's "bad", and output a pre-defined DC level until we can find some
         // good modem data. This flexibility on the DC level allows us to interact with the 
         // braking mode that might be set up in the decoder when DC levels are detected.
         // These decoder options often include the following:
         //    DC HIGH or LOW: Stop. Sometimes this is the decoder default, but might not be what you want!
         //    DC HIGH or LOW: Continue. This option is usually what we want since we don't want 
         //                    the loco to stop in some remote/possibly-inaccessible location (tunnels!)
         //    DC HIGH: continue; LOW: Stop. This is tricky stuff. This option is used to automatically stop locos
         //                                  on certain track sections. Might be useful since you can dynamically
         //                                  reset the DC level by accessing the AirMini on 9000 and set CV1017 
         //                                  to 0 (LOW) or non-zero (HIGH).
         if(((then-timeOfValidDCC) >= tooLong) && (getTransitionCount() > maxTransitionCount))
           {
             if (turnModemOnOff) strobeSPI(SIDLE);     // send stop command to modem if no DEMUX is available
             if(filterModemData) useModemData = 0;      // false use-of-modem-data state
             DCCuseModemData(useModemData,dcLevel);     // Tell the DCC code if you are or are not using modem data
             inactiveStartTime = then;                  // Start inactive timer
           }

           if(!useModemData) // If not using modem data, ensure the output is set to a DC level
             {
               if(dcLevel) OUTPUT_HIGH;             // HIGH
               else OUTPUT_LOW;                     // LOW
             }

      }
      // After sleeping for a while, wake up the modem and try to collect a valid DCC signal
      // Note the logic allows for possibility that the modem is left on 
      // and a valid DCC packet is found during the sleep time.
      else if(((sleepTime||turnModemOnOff) && ((then-inactiveStartTime) >= sleepTime)) || ((then-timeOfValidDCC) < tooLong))
      {
         useModemData = 1;                          // Active use-of-modem-data state
         DCCuseModemData(useModemData,dcLevel);     // Tell the DCC code if you are or are not using modem data
         timeOfValidDCC = then;                     // Start over on the DCC timing
         inactiveStartTime = then + BACKGROUNDTIME; // start the modem inactive timer sometime in the future
         if(turnModemOnOff)
         {
            dccInit();                             // Reset the DCC state machine, which also resets transitionCount
            strobeSPI(MODE);                       // awaken the modem in MODE
         }
         else resetTransitionCount(0);              // While we haven't reset the DCC state machine, do restart transitionCount
      }

      // Special processing for channel search
      if (initialWait) 
      {
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
            // If it's too long, then reset the modem to the next channel
            if (then > endInitialWaitTime) 
            {
#if defined(USE_LCD)
               if (LCDFound) LCD_Wait_Period_Over(0);
#endif
               if (++searchChannelIndex <= sizeof(searchChannels))
               // Keep searching...
               {
                  // Update the seach channel and endInitialWaitTime
                  CHANNEL = searchChannels[searchChannelIndex-1];
                  // Re-initialize the start of the wait time
                  startInitialWaitTime = then;      
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

               // Stop the modem
               strobeSPI(SIDLE);         
               // Reset the DCC state machine, which also resets transitionCount
               dccInit();                 
               // Restart on Airwire selected and mode (or power level)
               startModem(CHANNEL, MODE); 

            } // end of wait time over
         } // end of continue to wait
      } // End of special processing for channel search
//} end of RECEIVER
#endif

   } // end of if( (now - then) > BACKGROUNDTIME )

} // end of loop

///////////////////////////////////////////
// End of file AirMiniSketchTransmitter.ino 
///////////////////////////////////////////
