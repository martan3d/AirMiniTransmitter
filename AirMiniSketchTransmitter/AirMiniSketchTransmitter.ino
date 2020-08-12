/*
AirMiniSketchTransmitter.ino

Created: Dec  7 12:15:25 EST 2019

Copyright (c) 2019-2020, Martin Sant and Darrell Lamm
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

#include <EEPROM.h>
#include <dcc.h>
#include <spi.h>
#include <uart.h>
#include <schedule.h>
#include <servotimer.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <string.h>

#ifdef TRANSMIT
#undef RECEIVE
#define DCCLibrary
#define USE_LCD
#else
#define RECEIVE
#undef DCCLibrary
#undef USE_LCD
#define USE_LCD
#endif

#ifdef USE_LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#endif


#ifdef DCCLibrary
///////////////////
///////////////////
///////////////////
#include <DCCLibrary.h>
// Externs
extern bool (*GetNextMessage)(void); // For DCCLibrary
// For DCCLibrary
#ifdef TRANSMIT
#define DCC_PIN    2     // Arduino pin for DCC out to modem 
#else
#define DCC_PIN    4     // Arduino pin for DCC out, not currently used
#endif
#define DCC_DIAG0  5     // Diagnostic Pin #1
#define DCC_DIAG1  6     // Diagnostic Pin #2
#define msgSize 32       // The size of the ring buffer. Per Martin's new code
// Implement a ring buffer
volatile Message msg[msgSize] = {      // -> to DCCLibrary.c
    { { 0xFF, 0, 0xFF, 0, 0, 0}, 3},
    { { 0xFF, 0, 0xFF, 0, 0, 0}, 3},
    { { 0,    0, 0,    0, 0, 0}, 0}
  };      
// Private msg sent to DCCLibrary.c ISR
volatile Message msgExtracted[2] = {    // -> to DCCLibrary.c
    { { 0xFF, 0, 0xFF, 0, 0, 0}, 3}, // Will be overwritten in NextMessage
    { { 0xFF, 0, 0xFF, 0, 0, 0}, 3}, // The idle packet we will send in DCCLibrary.c when conditions permit/necessitate
};

// Idle message
const Message msgIdle = { { 0xFF, 0, 0xFF, 0, 0, 0}, 3};

volatile uint8_t lastMessageInserted  = 1;
volatile uint8_t lastMessageExtracted = 0;
volatile uint8_t currentIndex = 0;      // -> to DCCLibrary.c
uint8_t          newIndex     = 2;

///////////////////
///////////////////
///////////////////
#endif


// Times
#define INITIALDELAYMS    1000   // Initial processor start-up delay in ms
#define EEPROMDELAYMS      100   // Delay after eeprom write in ms
#define MILLISEC          4000   // @16Mhz, this is 1ms, 0.001s
#define QUARTERSEC     1000000   // @16Mhz, this is 0.25s
#define SEC            4000000   // @16Mhz, this is 1.00s
#define BACKGROUNDTIME   32000   // @16Mhz, this is 8ms

// CC1101 codes
#define RXMODE  0x34             // C1101 modem RX mode
#define RX      0x34
#define TX      0x35
#define STOP    0x36

// Setting up double pass for OPS mode
#ifdef TRANSMIT
#define DOUBLE_PASS 1            // Do a double pass on CV setting 
#else
#define DOUBLE_PASS 1            // Do a double pass on CV setting 
#endif

// DEFAULT defines
extern uint8_t channels_max;     // From spi.c
extern uint8_t channels_na_max;  // From spi.c

#ifdef TRANSMIT
#define POWERLEVELDEFAULT 8
#else
#define POWERLEVELDEFAULT 6
#endif

#define DCLEVEL_INDEFAULT 1
#define TURNMODEMON_INDEFAULT 0
#define IDLEPERIODMSDEFAULT 0
#define FILTERMODEMDATADEFAULT 0
#define AIRMINICV1DEFAULT 3

#if ! defined(AIRMINICV17DEFAULT)
#define AIRMINICV17DEFAULT 227
#endif
#pragma message "Info: Default CV17 is " xstr(AIRMINICV17DEFAULT)

#if ! defined(AIRMINICV18DEFAULT)
//{
#ifdef TRANSMIT
#define AIRMINICV18DEFAULT 40
#else
#define AIRMINICV18DEFAULT 41
#endif
//}
#endif
#pragma message "Info: Default CV18 is " xstr(AIRMINICV18DEFAULT)

#define AIRMINICV29DEFAULT 32

#ifdef RECEIVE
#define INITIALWAITPERIODSECDEFAULT 1
#else
//{
#if ! defined(AUTOIDLEOFFDEFAULT)
#define AUTOIDLEOFFDEFAULT 0
#endif
#pragma message "Info: Default AUTOIDLEOFFDEFAULT is " xstr(AUTOIDLEOFFDEFAULT)
//}
#endif


// Declarations
uint64_t now;
uint64_t then;

uint8_t sendbuffer[sizeof(DCC_MSG)];
uint8_t modemCVResetCount=0;
uint8_t dccptrAirMiniCVReset[sizeof(DCC_MSG)];
uint8_t dccptrNULL[sizeof(DCC_MSG)];
uint8_t * dccptr;
uint8_t dccptrRepeatCount = 0;
uint8_t dccptrRepeatCountMax = 2;
uint8_t j;

#ifdef TRANSMIT
uint8_t MODE = TX;                           // Mode is now a variable. Don't have the courage to change 
#else
uint8_t MODE = RX;                           // Mode is now a variable. Don't have the courage to change 
                                             // it in SW since you can't change it back in SW w/o a reset and 
                                             // using EEPROM data
#endif

uint8_t startModemFlag = 0;                  // Initial setting for calling startModem under some circumstances
uint8_t filterModemData;                     // Set the logical for whether to always use modem data. Initialized elsewhere
volatile uint8_t useModemData = 1;           // Initial setting for use-of-modem-data state
uint64_t idlePeriod = 0;                     // 0 msec, changed to variable that might be changed by SW
uint8_t idlePeriodms = 0;                    // 0 msec, changed to variable that might be changed by SW
uint64_t lastIdleTime = 0;
uint64_t tooLong  = 4000000;                 // 1 sec, changed to variable that might be changed by SW
uint64_t sleepTime = 0;                      // 0 sec, changed to variable that might be changed by SW
uint64_t timeOfValidDCC;                     // Time stamp of the last valid DCC packet
uint64_t inactiveStartTime;                  // Time stamp if when modem data is ignored
uint16_t maxTransitionCount;                 // Maximum number of bad transitions to tolerate before ignoring modem data
uint8_t maxTransitionCountLowByte=100;       // High byte of maxTransitionCount
uint8_t maxTransitionCountHighByte=0;        // Low byte of maxTransitionCount
#ifdef RECEIVE
uint8_t initialWait = 1;                     // Initial wait status for receiving valid DCC
uint64_t startInitialWaitTime;               // The start of the initial wait time. Will be set in initialization
uint64_t endInitialWaitTime;                 // The end of the initial wait time. Will be set in initialization
uint8_t InitialWaitPeriodSEC;                // Wait period
uint8_t searchChannelIndex = 0;              // Initialial channel search order index
#else
uint8_t AutoIdleOff;                         // Automatic Idle off. Will be intialized later
#endif

uint8_t bannerInit = 1;                      // Only display the banner the first time inside channel searching
uint8_t regionNum=0;
#if defined(NAEU_900MHz)
//{
#pragma message "Info: using European 869MHz/North American 915MHz frequency-dependent channels"
#ifdef RECEIVE
uint8_t searchChannels[18] = {0,17,16,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; // Channel search order
#endif
#ifdef USE_LCD
const char *bannerString = "ProMini Air NA/E";
const char *regionString[] = {"N","E"}; // Region code: N=North America, E=Europe, W=Worldwide
#endif
//}
#else
//{

#if defined(EU_434MHz)
//{
#pragma message "Info: using European 434MHz frequency-dependent channels"
#ifdef RECEIVE
uint8_t searchChannels[8] = {0,1,2,3,4,5,6,7}; // Channel search order
#endif
#ifdef USE_LCD
const char *bannerString = "ProMini Air EU";
const char *regionString[] = {"E"}; // Region code: N=North America, E=Europe, W=Worldwide
#endif
//}
#else
//{

#if defined(NAEU_2p4GHz)
//{
#pragma message "Info: using Worldwide 2.4GHz frequency-dependent channels"
#ifdef RECEIVE
uint8_t searchChannels[8] = {0,1,2,3,4,5,6,7}; // Channel search order
#endif
#ifdef USE_LCD
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

uint8_t AirMiniCV1;                          // The AirMini's address, HIGH byte

uint8_t AirMiniCV17;                         // The AirMini's address, HIGH byte
uint8_t AirMiniCV17tmp;                      // The AirMini's address, HIGH byte, temporary value until CV18 is reassigned in ops mode

uint8_t AirMiniCV18;                         // The AirMini's address, LOW byte

uint8_t AirMiniCV29;                         // The AirMini's address, HIGH byte
uint8_t AirMiniCV29Bit5;                     // The value of AirMiniCV29, bit 5
uint8_t useMyAddress = 0;                    // Global flag for LCD

// EEPROM data for persistence after turn-off of the AirMini
uint8_t  EEMEM EEisSetCHANNEL;               // Stored RF channel is set
uint8_t  EEMEM EECHANNEL;                    // Stored RF channel #
// uint8_t  EEMEM EECHANNELDefault;             // Stored RF channel #

uint8_t  EEMEM EEisSetturnModemOnOff;        // Stored modem turn on/off is set
uint8_t  EEMEM EEturnModemOnOff;             // Stored modem turn on/off
// uint8_t  EEMEM EEturnModemOnOffDefault;      // Stored modem turn on/off

uint8_t  EEMEM EEisSetdcLevel;               // Stored DC output level is set
uint8_t  EEMEM EEdcLevel;                    // Stored DC output level if modem turned off
// uint8_t  EEMEM EEdcLevelDefault;             // Stored DC output level if modem turned off

uint8_t  EEMEM EEisSetpowerLevel;            // Stored DC output power level is set
uint8_t  EEMEM EEpowerLevel;                 // Stored DC output power level 
// uint8_t  EEMEM EEpowerLevelDefault;          // Stored DC output power level 

uint8_t  EEMEM EEisSetidlePeriodms;          // Stored idlePeriodms set flag
uint8_t  EEMEM EEidlePeriodms;               // Stored idlePeriod in ms
// uint8_t  EEMEM EEidlePeriodmsDefault;        // Stored idlePeriod in ms

uint8_t  EEMEM EEisSetfilterModemData;       // Stored idlePeriodms set flag
uint8_t  EEMEM EEfilterModemData;            // Stored idlePeriod in ms
// uint8_t  EEMEM EEfilterModemDataDefault;     // Stored idlePeriod in ms

uint8_t  EEMEM EEisSetAirMiniCV1;            // Stored AirMini decoder short address is set
uint8_t  EEMEM EEAirMiniCV1;                 // Stored AirMini decoder short address
// uint8_t  EEMEM EEAirMiniCV1Default;          // Stored AirMini decoder short address

uint8_t  EEMEM EEisSetAirMiniCV17;           // Stored AirMini decoder high byte address is set
uint8_t  EEMEM EEAirMiniCV17;                // Stored AirMini decoder high byte address
// uint8_t  EEMEM EEAirMiniCV17Default;

uint8_t  EEMEM EEisSetAirMiniCV18;           // Stored AirMini decoder low byte address is set
uint8_t  EEMEM EEAirMiniCV18;                // Stored AirMini decoder low byte address
// uint8_t  EEMEM EEAirMiniCV18Default;         // Stored AirMini decoder low byte address

uint8_t  EEMEM EEisSetAirMiniCV29;           // Stored AirMini decoder configuration variable is set
uint8_t  EEMEM EEAirMiniCV29;                // Stored AirMini decoder configuration variable
// uint8_t  EEMEM EEAirMiniCV29Default;         // Stored AirMini decoder configuration variable

#ifdef RECEIVE
uint8_t  EEMEM EEisSetInitialWaitPeriodSEC;  // Stored AirMini decoder configuration variable
uint8_t  EEMEM EEInitialWaitPeriodSEC;       // Stored AirMini decoder configuration variable
// uint8_t  EEMEM EEInitialWaitPeriodSECDefault;// Stored AirMini decoder configuration variable
#else
uint8_t  EEMEM EEisSetAutoIdleOff;  // Stored AirMini decoder configuration variable
uint8_t  EEMEM EEAutoIdleOff;       // Stored AirMini decoder configuration variable
// uint8_t  EEMEM EEAutoIdleOffDefault;// Stored AirMini decoder configuration variable
#endif

enum {ACCEPTED, IGNORED, PENDING} CVStatus = ACCEPTED;

#ifdef USE_LCD
uint8_t EEMEM EEisSetLCDAddress;               // Stored LCD address set?
uint8_t EEMEM EELCDAddress;                    // Stored LCD address
// uint8_t EEMEM EELCDAddressDefault;             // Stored LCD address default
uint8_t LCDAddress;                            // The I2C address of the LCD
#define LCDCOLUMNS 16                          // Number of LCD columns
#define LCDROWS 2                              // Number of LCD rows 
uint64_t LCDTimePeriod=2*SEC;                  // Set up the LCD re-display time interval, 2 s
uint64_t prevLCDTime = 0;                      // Initialize the last time displayed
bool refreshLCD = true;                        // Whether to refresh
LiquidCrystal_I2C lcd;                         // Create the LCD object with a default address
char lcd_line[LCDCOLUMNS+1];                   // Note the "+1" to insert an end null!
#endif

/******************************************************************************/
#ifdef DCCLibrary
///////////////////
///////////////////
///////////////////
bool NextMessage(void){  // Sets currentIndex for DCCLibrary.c's access to msg
    
#if ! defined(DONTTURNOFFINTERRUPTS)
    cli(); // turn off interrupts
#endif
    bool retval = false;

    // Set the last message extracted from the ring buffer so DCCLibrary.c can use it
    if(lastMessageExtracted != lastMessageInserted)  // Update the extract pointer because we have not caught up the the inserted pointer
    {
       lastMessageExtracted = (lastMessageExtracted+1) % msgSize;
       retval = true;
    }

    currentIndex = lastMessageExtracted;                                              // Set the variable used by DCCLibrary.c to access the msg ring buffer with no update
    memcpy((void *)&msgExtracted[0], (void *)&msg[currentIndex], sizeof(Message));    // Extract the message into private msg
#if ! defined(DONTTURNOFFINTERRUPTS)
    sei(); // turn on interrupts
#endif

    return retval;
}
///////////////////
///////////////////
///////////////////
#endif

// Function to combine High and Low bytes into 16 byte variable
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

// Function, based on the value of forceDefault:
//    - TRUE:   TargetPtr's value and its related EEPROM variables are forced to use defaultValue
//    - FALSE:  extract and use EEPROM data, if previously-set, to set the TargetPtr's value
void checkSetDefaultEE(uint8_t *TargetPtr, const uint8_t *EEisSetTargetPtr, const uint8_t *EETargetPtr, uint8_t defaultValue, uint8_t forceDefault)
{
   uint8_t isSet; 
   if (EEisSetTargetPtr != (const uint8_t *)NULL) isSet = (uint8_t)eeprom_read_byte((const uint8_t *)EEisSetTargetPtr);
   else isSet = 0; // Bad if you get here!
   if (!isSet || forceDefault)
   {
      *TargetPtr = defaultValue; 
      eeprom_busy_wait();
      eeprom_update_byte( (uint8_t *)EEisSetTargetPtr, (const uint8_t)1 );
      delay(EEPROMDELAYMS); // Magic delay time to ensure update is complete
      eeprom_busy_wait();
      eeprom_update_byte( (uint8_t *)EETargetPtr, defaultValue );
      delay(EEPROMDELAYMS); // Magic delay time to ensure update is complete
   }
   else
   {
      if(EETargetPtr != (const uint8_t *)NULL) *TargetPtr = (uint8_t)eeprom_read_byte((const uint8_t *)EETargetPtr);
      else *TargetPtr = defaultValue;  // Bad if you get here!
   }

   eeprom_busy_wait();

}

#ifdef USE_LCD
void LCD_Banner(uint8_t bannerInit)
{
  lcd.setCursor(0,0);              // Set initial column, row
  if (bannerInit) lcd.print(bannerString);   // Banner
  else lcd.print("ProMini Air Info");
  lcd.setCursor(0,1);              // Set next line column, row
#ifdef TWENTY_SEVEN_MHZ
//{
  lcd.print("H:1.0 S:1.8/27MH");   // Show state
//}
#else
//{
#ifdef TWENTY_SIX_MHZ
  lcd.print("H:1.0 S:1.8/26MH");   // Show state
#else
//{
#error "Undefined crystal frequency"
//}
#endif
//}
#endif
  prevLCDTime  = getMsClock();     // Set up the previous display time
  refreshLCD = true;
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
  if(useMyAddress) 
  {
     useMyAddress = 0;
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
  else
  {
     useMyAddress = 1;
     uint8_t tmp = dccptr[0]&0b11000000;
     if ((tmp==0b11000000) && (dccptr[0]!=0b11111111))
     {
         int TargetAddress_int = ((int)dccptr[0]-192)*256+(int)dccptr[1];
         // snprintf(lcd_line,sizeof(lcd_line),"Msg Ad: %d(%d,%d)",TargetAddress_int,dccptr[0],dccptr[1]);
         snprintf(lcd_line,sizeof(lcd_line),"Msg Ad: %d(L)",TargetAddress_int);
     }
     else
     {
         snprintf(lcd_line,sizeof(lcd_line),"Msg Ad: %d(S)",(int)dccptr[0]);
     }
  }
    
  lcd.print(lcd_line);
  lcd.setCursor(0,1); // column, row

#if defined(NAEU_900MHz)
  if (CHANNEL <= channels_na_max)
     regionNum=0;
  else
     regionNum=1;
#endif

#ifdef TRANSMIT
  snprintf(lcd_line,sizeof(lcd_line),"Ch:%d(%s) PL:%d", CHANNEL, regionString[regionNum], powerLevel);
#else
  if (filterModemData) snprintf(lcd_line,sizeof(lcd_line),"Ch:%d(%s) Filt:%d", CHANNEL, regionString[regionNum], 1);
  else                 snprintf(lcd_line,sizeof(lcd_line),"Ch:%d(%s) Filt:%d", CHANNEL, regionString[regionNum], 0);
#endif
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
       snprintf(lcd_line,sizeof(lcd_line),"Changed CV%d=%d",CVnum,CVval);
    break;
    case IGNORED:
       snprintf(lcd_line,sizeof(lcd_line),"Ignored CV%d=%d",CVnum,CVval);
    break;
    case PENDING:
       snprintf(lcd_line,sizeof(lcd_line),"Pending CV%d=%d",CVnum,CVval);
    break;
  }
  lcd.print(lcd_line);
  prevLCDTime  = getMsClock();
  refreshLCD = true;
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
  prevLCDTime  = getMsClock();
  refreshLCD = true;

  // Display the banner one time after initialization since the first 
  // banner might go by too quickly
  if (bannerInit) 
  {
     bannerInit = 0;
     LCD_Banner(bannerInit);
  }

  return;
}
#endif

void setup() {

  delay(INITIALDELAYMS);

  DDRB |= 1;        // Use this for debugging if you wish
  initUART(38400);  // More debugging, send serial data out- decoded DCC packets

  ////////////////////////////////////////////////
  // Let's get the slow EEPROM stuff done first //
  ////////////////////////////////////////////////

  // Get the CHANNEL # stored in EEPROM and validate it
  // eeprom_update_byte(&EECHANNELDefault, CHANNELDEFAULT );
  checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, CHANNELDEFAULT, 0);      // Validate the channel, it's possible the EEPROM has bad data
  if(CHANNEL > channels_max) 
      checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, CHANNELDEFAULT, 1);  // Force the EEPROM data to use CHANNEL 0, if the CHANNEL is invalid
  
  // Just flat out set the powerLevel
  // eeprom_update_byte(&EEpowerLevelDefault, POWERLEVELDEFAULT );
  checkSetDefaultEE(&powerLevel, &EEisSetpowerLevel, &EEpowerLevel, POWERLEVELDEFAULT, 1);  // Force the reset of the power level. This is a conservative approach.

  // Set the alternate DC output level to HIGH or LOW (i.e., bad CC1101 data)
  // The level of this output can be used by some decoders. The default is HIGH.
  // eeprom_update_byte(&EEdcLevelDefault, DCLEVEL_INDEFAULT );
  checkSetDefaultEE(&dcLevel_in, &EEisSetdcLevel, &EEdcLevel, DCLEVEL_INDEFAULT, 0);       // Use EEPROM value if it's been set, otherwise set to 1 and set EEPROM values
  dcLevel = (volatile uint8_t)dcLevel_in;                                  // Since dcLevel is volatile we need a proxy uint8_t 

  // Turn the modem OFF/ON option. For use if bad modem data is detected in RX mode
  // eeprom_update_byte(&EEturnModemOnOffDefault, TURNMODEMON_INDEFAULT );
  checkSetDefaultEE(&turnModemOnOff_in, &EEisSetturnModemOnOff, &EEturnModemOnOff, TURNMODEMON_INDEFAULT, 0);  // Use EEPROM value if it's been set, otherwise set to 1 and set EEPROM values
  turnModemOnOff = (volatile uint8_t)turnModemOnOff_in;                                    // Needed to use a proxy variable since turnModemOnOff is volatile uint8_t

  // Set the DCC time-out period for sending IDLE packets. Used along with duplicate DCC packet detection for inserting IDLE packets
  // eeprom_update_byte(&EEidlePeriodmsDefault, IDLEPERIODMSDEFAULT );
  checkSetDefaultEE(&idlePeriodms, &EEisSetidlePeriodms, &EEidlePeriodms, IDLEPERIODMSDEFAULT, 0);  // Use EEPROM value if it's been set, otherwise set to 0 ms
  idlePeriod = (uint64_t)idlePeriodms * MILLISEC;                                 // Convert to time counts

  // Set whether to always use modem data on transmit
  // eeprom_update_byte(&EEfilterModemDataDefault, FILTERMODEMDATADEFAULT );
  checkSetDefaultEE(&filterModemData, &EEisSetfilterModemData, &EEfilterModemData, FILTERMODEMDATADEFAULT, 0);  // Use EEPROM value if it's been set, otherwise set to 0 

  // Get up addressing-related CV's from EEPROM, or if not set set them in EEPROM
  // eeprom_update_byte(&EEAirMiniCV1Default, AIRMINICV1DEFAULT );
  checkSetDefaultEE(&AirMiniCV1,  &EEisSetAirMiniCV1,  &EEAirMiniCV1,    AIRMINICV1DEFAULT, 0);  // Short address. By default, not using

  // eeprom_update_byte(&EEAirMiniCV17Default, AIRMINICV17DEFAULT );
  checkSetDefaultEE(&AirMiniCV17, &EEisSetAirMiniCV17, &EEAirMiniCV17, AIRMINICV17DEFAULT, 0);  // High byte to set final address to 9000
  AirMiniCV17tmp = AirMiniCV17;                                                  // Due to the special nature of CV17 paired with CV18

  // eeprom_update_byte(&EEAirMiniCV18Default, AIRMINICV18DEFAULT );
  checkSetDefaultEE(&AirMiniCV18, &EEisSetAirMiniCV18, &EEAirMiniCV18, AIRMINICV18DEFAULT, 0);  // Low byte to set final address to 9000/9001 for transmitter/receiver

  // eeprom_update_byte(&EEAirMiniCV29Default, AIRMINICV29DEFAULT );
  checkSetDefaultEE(&AirMiniCV29, &EEisSetAirMiniCV29, &EEAirMiniCV29,  AIRMINICV29DEFAULT, 0);  // Set CV29 so that it will use a long address
  AirMiniCV29Bit5 = AirMiniCV29 & 0b00100000;                                    // Save the bit 5 value of CV29 (0: Short address, 1: Long address)

#ifdef RECEIVE
  // eeprom_update_byte(&EEInitialWaitPeriodSECDefault, INITIALWAITPERIODSECDEFAULT );
  checkSetDefaultEE(&InitialWaitPeriodSEC, &EEisSetInitialWaitPeriodSEC, &EEInitialWaitPeriodSEC,  INITIALWAITPERIODSECDEFAULT, 0);  // Wait time in sec
#else
  // eeprom_update_byte(&EEAutoIdleOffDefault, AUTOIDLEOFFDEFAULT );
  checkSetDefaultEE(&AutoIdleOff, &EEisSetAutoIdleOff, &EEAutoIdleOff,  AUTOIDLEOFFDEFAULT, 0);  // Set AutoIdleOff
#endif
#ifdef USE_LCD
  // eeprom_update_byte(&EELCDAddressDefault, LCDADDRESSDEFAULT );
  checkSetDefaultEE(&LCDAddress, &EEisSetLCDAddress, &EELCDAddress,  LCDADDRESSDEFAULT, 0);  // Set LCDAddress
#endif

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

  // Set up slow-time variables
  then = getMsClock();                        // Grab Current Clock value for the loop below
  timeOfValidDCC = then;                      // Initialize the valid DCC data time
#ifdef RECEIVE
  initialWait = 1;
  startInitialWaitTime = timeOfValidDCC;      // Initialize the start of the wait time (never is not a good value)
  endInitialWaitTime = 
                    startInitialWaitTime
                  + InitialWaitPeriodSEC*SEC; // Initialize the end of the wait time
#endif
  inactiveStartTime = then + BACKGROUNDTIME;  // Initialize the modem idle time into the future

  ///////////////////////////////////////////////
  // Set up the hardware and related variables //
  ///////////////////////////////////////////////

  // Set up and initialize the output of the diagnostic pin (done in dccInit)
  // SET_OUTPUTPIN;                           // Set up the output diagnostic DCC pin. This is our filtered output in Rx mode
  // if(dcLevel) OUTPUT_HIGH;                 // HIGH
  // else OUTPUT_LOW;                         // LOW

#ifdef USE_LCD
  lcd.init(LCDAddress,LCDCOLUMNS,LCDROWS);    // Initialize the LCD
  lcd.backlight();                            // Backlight it
  LCD_Banner(bannerInit);                     // Display the banner on LCD
#endif

#ifdef DCCLibrary
   ////////////////////
   // For DCCLibrary //
   ////////////////////
   GetNextMessage = &NextMessage;             // assign a proper function to GetNextMessage that actually does something

   //Set the pins for DCC to "output".
   pinMode(DCC_PIN,OUTPUT);                   // this is for the DCC Signal output to the modem for transmission
    
   pinMode(DCC_DIAG0,OUTPUT);                 // 
   digitalWrite(DCC_DIAG0,0);                 // Will use this for diagnostics

   pinMode(DCC_DIAG1,OUTPUT);                 //
   digitalWrite(DCC_DIAG1,0);                 // Will use this for diagnostics

   SetupDCC(DCC_PIN);   
   /////////////////////
   /////////////////////
   /////////////////////
 #endif

  initServoTimer();                           // Master Timer plus servo timer
  initializeSPI();                            // Initialize the SPI interface to the radio
  delay(10);                                  // Wait a bit for the SPI
  dccInit();                                  // Enable DCC transmit/receive
  startModem(CHANNEL, MODE);                  // Start on this Channel

  sei();                                      // enable interrupts

}
// End of setup


void loop() {


        /* Check High Priority Tasks First */

        switch( masterSchedule() )
         {
             case TASK0:                      // Highest Priority Task goes here
                                              // We don't have one right now so this is just a placeholder
                     clearScheduledTask(TASK0);
                  break;

             case TASK1:                      // Just pick a priority for the DCC packet, TASK1 will do 

                     dccptr = getDCC();       // we are here, so a packet has been assembled, get a pointer to our DCC data

                     if (memcmp(sendbuffer,dccptr,sizeof(DCC_MSG))) dccptrRepeatCount=0;  // If they don't match, reset the repeat count
                     else dccptrRepeatCount++;                                            // If they do match, increment the repeat count

                     decodeDCCPacket((DCC_MSG*) dccptr);      // Send debug data
                     memcpy(sendbuffer,dccptr,sizeof(DCC_MSG));

#ifdef TRANSMIT
//{

#if ! defined(DONTTURNOFFINTERRUPTS)
                     cli(); // Turn off interrupts
#endif

#ifdef DCCLibrary
                     newIndex = (lastMessageInserted+1) % msgSize;  // Set the last message inserted into the ring buffer 
#endif

                     // Logic to pass through packet or send an IDLE packet to keep Airwire keep-alive
                     if((dccptrRepeatCount < dccptrRepeatCountMax) || (((uint64_t)getMsClock() - lastIdleTime) < idlePeriod) || AutoIdleOff) 
                     {                     
#ifdef DCCLibrary
                        memcpy((void *)&msg[newIndex],(void *)dccptr,sizeof(DCC_MSG)); // Dangerous, fast copy
#endif
                     }
                     else                                       // Send out an idle packet (for keep-alive!)
                     {
                        dccptrRepeatCount = 0;
                        lastIdleTime = getMsClock();
#ifdef DCCLibrary
                        memcpy((void *)&msg[newIndex],(void *)&msgIdle,sizeof(DCC_MSG)); // Dangerous, fast copy
#endif
                     }

#ifdef DCCLibrary
                     // Update the last message inserted for comparisons. 
                     // We do it here to make sure the ISR's don't affect the value
                     lastMessageInserted = newIndex;
#endif
#if ! defined(DONTTURNOFFINTERRUPTS)
                     sei(); // Turn interrupts back on
#endif
//} TRANSMIT
#endif

#ifdef RECEIVE
                    if(!useModemData) // If not using modem data, ensure that the output is set to a DC level after coming back from the ISR
                      {
                        if(dcLevel) OUTPUT_HIGH;                // HIGH
                        else OUTPUT_LOW;                        // LOW
                      }
#endif

                   /////////////////////////////////////////////
                   // Special processing for AirMini OPS mode //
                   /////////////////////////////////////////////
                   if(((dccptr[0]==AirMiniCV17) && (dccptr[1]== AirMiniCV18) &&  AirMiniCV29Bit5) ||
                      ((dccptr[0]==AirMiniCV1)                               && !AirMiniCV29Bit5) )
                     {
                       // According the NMRA standards, two identical packets should be received
                       // before modifying CV values. This feature now works (i.e., DOUBLE_PASS=1(=true)).
                       uint8_t countPtr = 1;
                       if (AirMiniCV29Bit5) countPtr = 2;
                       uint8_t tmpuint8 = dccptr[countPtr]&(0b11111100); // The last two bits are part of the CV address used below and we don't care right now.
                                                                         // Do NOT increment countPtr because we aren't finished withe dccptr[countPtr] yet;
                                                                         // we needs its two low bytes for the upper two bytes of the CV address below!
                       if(tmpuint8==0b11101100)                          // Determine if the bit pattern is for modifying CV's with the last two bits don't care
                         {
                            if(modemCVResetCount==0 && DOUBLE_PASS)                   // Processing for identifying first or second valid call
                              {
                                 modemCVResetCount++;                                 // Update the CV reset counter
                                 memcpy(dccptrAirMiniCVReset,dccptr,sizeof(DCC_MSG)); // Save the dcc packet for comparison
                              }
                            else 
                              {
                                 if(!memcmp(dccptrAirMiniCVReset,dccptr,sizeof(DCC_MSG)) || !DOUBLE_PASS)  // If they don't compare, break out
                                   {
                                    startModemFlag = 0;             // Initialize whether the modem will be restarted
                                    tmpuint8 = dccptr[countPtr++]&(0b00000011); // zero out the first 6 bits of dccptr[countPtr], we want to use the last two bits
                                    CVnum = (uint16_t)tmpuint8;     // cast the result to a 2 byte CVnum
                                    CVnum <<= 8;                    // now move these two bit over 8 places, now that it's two bytes
                                    uint16_t tmpuint16 = (uint16_t)dccptr[countPtr++]; // cast dccptr[countPtr] to 2 bytes
                                    CVnum |= tmpuint16;             // set the last 8 bits with dccptr[countPtr]
                                    CVnum++;                        // NMRA Std of plus one, good grief, to set the final CV number
                                    CVval = dccptr[countPtr++];     // Set CVval to dccptr[countPtr], one byte only!
                                    CVStatus = ACCEPTED;            // Set the default CV status

	                            switch(CVnum)
                                    {
                                      case  255:  // Set the channel number and reset related EEPROM values. Modest error checking. Verified this feature works
                                          if(CVval <= channels_max)           // Check for good values
                                            {
                                              checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, CVval, 1);  
                                              startModemFlag = 1;
                                            }
                                          else                      // Ignore bad values
                                            CVStatus = IGNORED;
                                      break;
                                      case  254:  // Set the RF power level and reset related EEPROM values. Verified this feature works.
                                          if(CVval<=10) 
                                            {
                                              checkSetDefaultEE(&powerLevel, &EEisSetpowerLevel, &EEpowerLevel, CVval, 1); // Set powerLevel and reset EEPROM values. Ignore bad values
                                              startModemFlag = 1;
                                            }
                                          else
                                            CVStatus = IGNORED;
                                      break;
                                      case  253:  // Turn off/on the modem for bad packet intervals and reset related EEPROM values. Verified this feature works
                                          checkSetDefaultEE(&turnModemOnOff_in, &EEisSetturnModemOnOff, &EEturnModemOnOff, CVval, 1); // Set turnModemOnOff and reset EEPROM values
                                          turnModemOnOff = (volatile uint8_t)turnModemOnOff_in; // Assign for volatile
                                      break;
                                      case  252:  // Set the tooLong (in quarter second intervals) and reset related EEPROM values. 
                                          tooLong = CVval*QUARTERSEC;
                                      break;
                                      case  251:  // Set the sleepTime (in quarter second intervals) and reset related EEPROM values. Verified this feature works.
                                          sleepTime = CVval*QUARTERSEC;
                                      break;
                                      case  250:  // Set the low byte for transition counts
                                          maxTransitionCountLowByte = CVval;
                                          maxTransitionCount = combineHighLow(maxTransitionCountHighByte,maxTransitionCountLowByte);
                                      break;
                                      case  249:  // Set the high byte for transition counts
                                          maxTransitionCountHighByte = CVval;
                                          maxTransitionCount = combineHighLow(maxTransitionCountHighByte,maxTransitionCountLowByte);
                                      break;
                                      case  248:  // Set the DC output level and reset related EEPROM values. Verified this feature works.
                                          checkSetDefaultEE(&dcLevel_in, &EEisSetdcLevel, &EEdcLevel, CVval, 1); // Set dcLevel and reset EEPROM values
                                          dcLevel = (volatile uint8_t)dcLevel_in;
                                      break;
                                      case  247:  // Set the idle period (in ms) and reset related EEPROM values. Verified it works.
                                          checkSetDefaultEE(&idlePeriodms, &EEisSetidlePeriodms, &EEidlePeriodms, CVval, 1); // Set idlePeriodms and reset EEPROM values (in ms!)
                                          idlePeriod = idlePeriodms * MILLISEC; // Convert to cycles
                                      break;
                                      case  246:  // Set whether to always use modem data
                                          checkSetDefaultEE(&filterModemData, &EEisSetfilterModemData, &EEfilterModemData, CVval, 1); // Set filterModemData and reset EEPROM values
                                      break;
#ifdef RECEIVE
                                      case  245:  // Set the wait period in 1 second intervals - Nothing can be done with this until reset
                                          if(CVval <= 60)
                                             checkSetDefaultEE(&InitialWaitPeriodSEC, &EEisSetInitialWaitPeriodSEC, &EEInitialWaitPeriodSEC,  CVval, 1);  // Wait time in sec
                                          else
                                            CVStatus = IGNORED;
                                      break;
#endif
#ifdef TRANSMIT
                                      case  244:  // Turn off automatic IDLE insertion
                                           checkSetDefaultEE(&AutoIdleOff, &EEisSetAutoIdleOff, &EEAutoIdleOff,  CVval, 1); 
                                      break;
#endif
#ifdef USE_LCD
                                      case  243:  // Reset the LCD address
                                           checkSetDefaultEE(&LCDAddress, &EEisSetLCDAddress, &EELCDAddress,  CVval, 1);  // Set LCD address for the NEXT boot
                                           CVStatus = PENDING;
                                      break;
#endif
                                      case 29:    // Set the Configuration CV and reset related EEPROM values. Verified this feature works.
                                          checkSetDefaultEE(&AirMiniCV29, &EEisSetAirMiniCV29, &EEAirMiniCV29, CVval, 1); 
                                          AirMiniCV29Bit5 = AirMiniCV29 & 0b00100000; // Save the bit 5 value of CV29 (0: Short address, 1: Long address)
                                      break;
                                      case 18:    // Set the Long Address Low Byte CV and reset related EEPROM values. Verified this feature works.
                                                  // See NMRA S-9.2.1 Footnote 8.
                                          checkSetDefaultEE(&AirMiniCV17, &EEisSetAirMiniCV17, &EEAirMiniCV17, AirMiniCV17tmp, 1); 
                                          checkSetDefaultEE(&AirMiniCV18, &EEisSetAirMiniCV18, &EEAirMiniCV18, CVval, 1); 
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
                                           if (CVval==8) {
#ifdef USE_LCD
                                              lcd.clear();
                                              lcd.setCursor(0,0); // column, row
                                              snprintf(lcd_line,sizeof(lcd_line),"Keep Power ON!");
                                              lcd.print(lcd_line);
                                              lcd.setCursor(0,1); // column, row
                                              snprintf(lcd_line,sizeof(lcd_line),"Factory Reset...");
                                              lcd.print(lcd_line);
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
                                              checkSetDefaultEE(&AirMiniCV1, &EEisSetAirMiniCV1, &EEAirMiniCV1, CVval, 1); 
                                            }
                                          else
                                            CVStatus = IGNORED;
                                      break;
                                      default:
                                          CVStatus = IGNORED;
                                      break;
                                    } // end of switch(CVnum) 

#ifdef USE_LCD
                                    LCD_CVval_Status(CVnum,CVval);
#endif

                                    if(startModemFlag)
                                      {
                                         sendReceive(STOP);         // Stop the modem
                                         dccInit();                 // Reset the DCC state machine, which also resets transitionCount
                                         startModem(CHANNEL, MODE); // Restart on possible-new Airwire Channel and mode (or power level)
                                      }


                                 } // end of if(!memcmp...

                                 modemCVResetCount=0;                                     // Reset the CV reset counter
                                 memcpy(dccptrAirMiniCVReset,dccptrNULL,sizeof(DCC_MSG)); // Reset the dcc packet for comparison

                              } // end of else (modemCVResetCount ...

                         } // end of if(tmpuint8 ...
                       else  // Not in OPS mode
                         {
                           modemCVResetCount=0;           // Reset this counter if we didn't get a "hit" on CV changing
                         } // End of not in OPS mode

                     } // end of if((dccptr[0] ...
                   ///////////////////////////////////////////////////
                   // End ofSpecial processing for AirMini OPS mode //
                   ///////////////////////////////////////////////////

                   timeOfValidDCC = getMsClock();  // Grab Current Clock value for the checking below
                   clearScheduledTask(TASK1);      // all done, come back next time we are needed
  
                  break; // TASK1 break
         } // End of switch( masterSchedule() )

         /**** After checking highest priority stuff, check for the timed tasks ****/

        now = getMsClock() - then;             // How long has it been since we have come by last?

#ifdef RECEIVE
        if(!useModemData)        // If not using modem data, ensure the output is set to DC after coming back from the ISR
          {
            if(dcLevel) OUTPUT_HIGH;           // HIGH
            else OUTPUT_LOW;                   // LOW
          }
#endif
        
         if( now > BACKGROUNDTIME )            // Check for Time Scheduled Tasks
           {                                   // A priority Schedule could be implemented in here if needed
              then = getMsClock();             // Grab Clock Value for next time

#ifdef USE_LCD
              if(refreshLCD && ((then-prevLCDTime) >= LCDTimePeriod))
                {
                  LCD_Addr_Ch_PL();                // Update the display of address, chanel #, and power level
                  prevLCDTime = then;              // Slowly... at 1 sec intervals
                  refreshLCD = true;
                }
#endif

#ifdef TRANSMIT
              sendReceive(MODE);         // keep the radio awake in MODE 
#else
              if(useModemData)
                {
                   sendReceive(MODE);         // keep the radio awake in MODE 

                   // If the DCC data collection appears hung up, put the modem to sleep (if we want to)
                   // and just ignore its data for a while...
                   // The modem's output will "stick" to either LOW or HIGH (it's random!) when turned off, 
                   // causing the amplifier to go LOW or HIGH. The Airwire forces output high when no RF 
                   // (or really keep-alive) is received. We will simply ignore the modem data when
                   // we think it's "bad", and output a pre-defined DC level util we can find some
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
                       if (turnModemOnOff) sendReceive(STOP);     // send stop command to modem if no DEMUX is available
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
                       sendReceive(MODE);                     // awaken the modem in MODE
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
#ifdef USE_LCD
                    LCD_Wait_Period_Over(1);
#endif
                 }
                 else  // Othewise, continue to wait
                 {
                    // If it's too long, then reset the modem to the next channel
                    if (then > endInitialWaitTime) 
                    {
#ifdef USE_LCD
                       LCD_Wait_Period_Over(0);
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
                              + InitialWaitPeriodSEC*SEC; 
                       }
                       else 
                       // Last resort
                       {
                          initialWait = 0;
                          CHANNEL=CHANNELDEFAULT;    // Reset to the last resort channel
                       }

                       // Stop the modem
                       sendReceive(STOP);         
                       // Reset the DCC state machine, which also resets transitionCount
                       dccInit();                 
                       // Restart on Airwire selected and mode (or power level)
                       startModem(CHANNEL, MODE); 

                    } // end of wait time over
                 } // end of continue to wait
              } // End of special processing for channel search
#endif

              PORTB ^= 1;                      // debug - monitor with logic analyzer
          }
}
