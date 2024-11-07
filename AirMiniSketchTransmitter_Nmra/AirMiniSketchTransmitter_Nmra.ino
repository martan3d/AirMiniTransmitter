/* 
AirMiniSketchTransmitter_Nmra.ino 
S:1.7Z:
- Keep output off until the channel scanning is finished.

Created: Jun 6 2021 using AirMiniSketchTransmitter.ino
        as a starting point

Copyright (c) 2021-2024, Darrell Lamm
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

#include <config.h>
#include <EEPROM.h>
#include <spi.h>
#include <uart.h>
#include <schedule.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <string.h>
#include <NmraDcc.h>

#define HWVERSION "2"
#pragma message "Info: Hardware version is " xstr(HWVERSION)
#define SWVERSION "1.7Z"
#pragma message "Info: Software version is " xstr(SWVERSION)

#if defined(TWENTY_SEVEN_MHZ)
//{
#define FREQMH "27"
//}
#else
//{
#if defined(TWENTY_SIX_MHZ)
//{
#define FREQMH "26"
//}
#else
//{
#error "Undefined crystal frequency"
//}
#endif
//}
#endif
#pragma message "Info: FREQMH is " xstr(FREQMH)

#undef DEBUG_LOCAL
#if defined(TURNOFFINTERRUPTS)
#pragma message "Turning off interrupts in message re-assigment sections"
#else
#pragma message "NOT turning off interrupts in message re-assigment sections"
#endif

#define USE_NEW_LCD
#if ! defined(USE_NEW_LCD)
#define USE_OLD_LCD
#endif

#if defined(USE_NEW_LCD)
#pragma message "Using OLED display"
#define LCD_PRINT lcd.println
#else
#pragma message "Using LCD display"
#define LCD_PRINT lcd.print
#endif

#if defined(TRANSMITTER)
#undef RECEIVER
#elif defined(RECEIVER)
#else
#error "Error: Neither TRANSMITTER or RECEIVER is defined"
#endif

#if defined(USE_OLD_LCD)
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#elif defined(USE_NEW_LCD)
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#endif

#if defined(TRANSMITTER)
//{ TRANSMITTER

// Actual input pin. No conversion do PD3! DCC.init does this.
// #define INPUT_PIN  PD3  // 5V unipolar DCC input from opto-coupler
#define INPUT_PIN 3
#define EXTINT_NUM 1
// #define OUTPUT_PIN1 PD4
// Output to CC1101 modem (GD0)
#define OUTPUT_PIN1 PD2

#define OUTPUT_HIGH   PORTD = PORTD |  (1 << OUTPUT_PIN1)
#define OUTPUT_LOW    PORTD = PORTD & ~(1 << OUTPUT_PIN1)
#define SET_OUTPUTPIN DDRD |= (1 << OUTPUT_PIN1)

//} TRANSMITTER
#else
//{ RECEIVER

// Actual input pin. No conversion do PD3! DCC.init does this.
// #define INPUT_PIN  PD2  // 3.3V unipolar DCC input from raw modem output
#define INPUT_PIN 2
#define EXTINT_NUM 0
#define OUTPUT_PIN1 PD3
#define OUTPUT_PIN2 PD4

#define OUTPUT_OFF    PORTD = (PORTD &  ~(1 << OUTPUT_PIN1)) &  ~(1 << OUTPUT_PIN2)
#define OUTPUT_HIGH   PORTD = (lockedAntiphase ? (PORTD | (1 << OUTPUT_PIN2)) |  (1 << OUTPUT_PIN1) : \
(PORTD |  (1 << OUTPUT_PIN1)) & ~(1 << OUTPUT_PIN2))
#define OUTPUT_LOW    PORTD = (lockedAntiphase ? (PORTD | (1 << OUTPUT_PIN2)) & ~(1 << OUTPUT_PIN1) : \
(PORTD & ~(1 << OUTPUT_PIN1)) |  (1 << OUTPUT_PIN2))
#define SET_OUTPUTPIN DDRD  = (DDRD | (1 << OUTPUT_PIN1)) | (1 << OUTPUT_PIN2)

//} RECEIVER
#endif

NmraDcc DCC;

//              100 -> 6.25uS @ 16MHz
#define ADVANCE 200
volatile uint16_t advance = ADVANCE;
// Timer frequency is 16MHz for ( /1 prescale from 16MHz )
#define TIMER_SHORT 64608  // 58usec pulse length
#if !defined(TIMER_LONG)
#define TIMER_LONG 63680  // 116usec pulse length
#endif
volatile uint16_t timer_long  = TIMER_LONG;
volatile uint16_t timer_short = TIMER_SHORT;
#if defined(PRINT_LATENCY)
volatile uint16_t latency = 0;
#endif
#define sleep() asm volatile ("nop")

// definitions for state machine
// uint8_t last_timer = timer_short;  // store last timer value
// uint8_t timer_val = timer_long;  // The timer value
volatile uint16_t timer_val = timer_short;  // The timer value
volatile uint8_t every_second_isr = 0;  // pulse up or down
volatile uint8_t num_cutout = 0;
#if ! defined(MAX_NUM_CUTOUT)
#define MAX_NUM_CUTOUT 20
#endif
#pragma message "Info: MAX_NUM_CUTOUT: " xstr(MAX_NUM_CUTOUT)

volatile enum {PREAMBLE, STARTBYTE, SENDBYTE, STOPPACKET, CUTOUT} current_state, next_state = PREAMBLE;
#define MINIMUM_PREAMBLE_BITS 16
#define MAXIMUM_PREAMBLE_BITS 30
#if defined(TRANSMITTER)
#if !defined(PREAMBLE_BITS)
#define PREAMBLE_BITS MAXIMUM_PREAMBLE_BITS
#endif
uint8_t preamble_bits = PREAMBLE_BITS;  // Large enough for Airwire
#pragma message "Info: Transmitter PREAMBLE_BITS is " xstr(PREAMBLE_BITS)
#endif
volatile uint8_t preamble_count = MINIMUM_PREAMBLE_BITS; // will be reset in the ISR
volatile uint8_t outbyte = 0;
volatile uint8_t cbit = 0x80;
volatile uint8_t byteIndex = 0;

// For NmraDcc
#define OUTPUT_ENABLE 5  // Output Enable
#define DCC_DIAG1 6      // Diagnostic Pin #2
#define MAXMSG 16        // The size of the ring buffer. Per Martin's new code
// Implement a ring buffer
volatile DCC_MSG msg[MAXMSG] = {
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
  { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}},
};

// Idle message
const DCC_MSG msgIdle = { 3, MINIMUM_PREAMBLE_BITS, { 0xFF, 0, 0xFF, 0, 0, 0}};  // idle msg

volatile uint8_t msgIndexOut = 0;
volatile uint8_t msgIndexIn  = 0;  // runs from 0 to MAXMSG-1

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

// DEFAULT defines
extern uint8_t channels_max;     // From spi.c
extern uint8_t channels_na_max;  // From spi.c

#if defined(TRANSMITTER)
#define POWERLEVELDEFAULT 8
#else
#define POWERLEVELDEFAULT 6
#endif

#if !defined(LOCKEDANTIPHASEDEFAULT)
#define LOCKEDANTIPHASEDEFAULT 1
#endif
#pragma message "Info: default lockedAntiphase is " xstr(LOCKEDANTIPHASEDEFAULT)

#define IDLEPERIODMSDEFAULT 128

#if ! defined(FILTERMODEMDATADEFAULT)
#define FILTERMODEMDATADEFAULT 1
#endif
#pragma message "Info: default filtering modem data value is " xstr(FILTERMODEMDATADEFAULT)

#if ! defined(DCLEVELDEFAULT)
#define DCLEVELDEFAULT 1
#endif
#pragma message "Info: default output 1 DC level value is " xstr(DCLEVELDEFAULT)


#define AIRMINICV1DEFAULT 3

#if !defined(AIRMINICV17DEFAULT)
#define AIRMINICV17DEFAULT 230
#endif
#pragma message "Info: default CV17 is " xstr(AIRMINICV17DEFAULT)

#if !defined(AIRMINICV18DEFAULT)
//{
#if defined(TRANSMITTER)
#define AIRMINICV18DEFAULT 172
#else
#define AIRMINICV18DEFAULT 173
#endif
//}
#endif
#pragma message "Info: default CV18 is " xstr(AIRMINICV18DEFAULT)

#define AIRMINICV29DEFAULT 32

#if defined(RECEIVER)
//{ RECEIVER
#define INITIALWAITPERIODSECDEFAULT 1
//} RECEIVER
#else
//{ TRANSMITTER
#if !defined(AUTOIDLEOFFDEFAULT)
#define AUTOIDLEOFFDEFAULT 1
#endif
#pragma message "Info: default AUTOIDLEOFFDEFAULT is " xstr(AUTOIDLEOFFDEFAULT)
//} TRANSMITTER
#endif

// Declarations
uint64_t now;
uint64_t then;

volatile DCC_MSG *dccptrIn  = (volatile DCC_MSG *)&msgIdle;
volatile DCC_MSG *dccptrTmp = (volatile DCC_MSG *)&msgIdle;
volatile DCC_MSG *dccptrOut = (volatile DCC_MSG *)&msgIdle;
volatile DCC_MSG *dccptrISR = (volatile DCC_MSG *)&msgIdle;
volatile bool printIn = true;

uint8_t dccptrNULL[sizeof(DCC_MSG)];

#if defined(TRANSMITTER)
uint8_t MODE = TX;                           // Mode is now a variable. Don't have the courage to change
#else
uint8_t MODE = RX;                           // Mode is now a variable. Don't have the courage to change
                                             // it in SW since you can't change it back in SW w/o a reset and
                                             // using EEPROM data
#endif

uint8_t startModemFlag = 0;                  // Initial setting for calling startModem under some circumstances
uint8_t filterModemData = 1;                 // Set the logical for whether to always use modem data.
                                             // Initialized elsewhere

volatile uint8_t useModemData = 1;           // Initial setting for use-of-modem-data state
volatile uint64_t timeOfValidDCC;            // Time stamp of the last valid DCC packet
#if ! defined(TOO_LONG)
#define TOO_LONG 16
#endif
#pragma message "Info: TOO_LONG interval (number of 1/4 sec intervals): " xstr(TOO_LONG)
uint64_t tooLong = (uint64_t)(TOO_LONG)*QUARTERSEC;         // 1/4 sec, changed to variable that might be changed by SW

#if defined(RECEIVER)
//{ RECEIVER
uint64_t startInitialWaitTime;               // The start of the initial wait time. Will be set in initialization
uint64_t endInitialWaitTime;                 // The end of the initial wait time. Will be set in initialization
uint8_t InitialWaitPeriodSEC;                // Wait period
uint8_t searchChannelIndex = 0;              // Initialial channel search order index
//} RECEIVER
#else
//{ TRANSMITTER
//} TRANSMITTER
#endif

#if defined(TRANSMITTER)
uint8_t initialWait = 0;                     // Initial wait status for receiving valid DCC
#else
uint8_t initialWait = 1;                     // Initial wait status for receiving valid DCC
#endif

enum {INITIAL, INFO, NONE} whichBanner = INITIAL;
uint8_t regionNum = 0;
#if defined(NAEU_900MHz)
//{
#pragma message "Info: using European 869MHz/North American 915MHz frequency-dependent channels"

#if defined(RECEIVER)
uint8_t searchChannels[19] = {0, 18, 17, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};  // Channel search
                                                                                                  // order
#endif

#if defined(USE_OLD_LCD) || defined(USE_NEW_LCD)
const char *bannerString = "ProMini Air NA/E";
const char *regionString[] = {"N", "E"};  // Region code: N=North America, E=Europe, W=Worldwide
#endif
//}
#else
//{

#if defined(EU_434MHz)
//{
#pragma message "Info: using European 434MHz frequency-dependent channels"

#if defined(RECEIVER)
uint8_t searchChannels[8] = {0, 1, 2, 3, 4, 5, 6, 7};  // Channel search order
#endif

#if defined(USE_OLD_LCD) || defined(USE_NEW_LCD)
const char *bannerString = "ProMini Air EU";
const char *regionString[] = {"E"};  // Region code: N=North America, E=Europe, W=Worldwide
#endif
//}
#else
//{

#if defined(NAEU_2p4GHz)
//{
#pragma message "Info: using Worldwide 2.4GHz frequency-dependent channels"

#if defined(RECEIVER)
uint8_t searchChannels[8] = {0, 1, 2, 3, 4, 5, 6, 7};  // Channel search order
#endif

#if defined(USE_OLD_LCD) || defined(USE_NEW_LCD)
const char *bannerString = "ProMini Air WW";
const char *regionString[] = {"W"};  // Region code: N=North America, E=Europe, W=Worldwide
#endif
//}
#endif

//}
#endif

//}
#endif

// Changing with CV's
// uint16_t CVnum;                              // CV numbers consume 10 bits
// uint8_t CVval;                               // CV values consume only 8 bits
uint8_t CHANNEL;                             // Airwire Channel for both TX/RX, may change in SW. Do NOT initialize
uint8_t lockedAntiphase;                     // Do NOT intialize, in EEPROM
volatile uint8_t dcLevel;                    // The output level (HIGH or LOW) output if modem data is invalid
extern uint8_t powerLevel;                   // The modem power level (>=0 and <=10). Communicated to spi.c
extern uint8_t deviatnval;                   // FSK deviation hex code
extern uint8_t deviatn_changed;              // Deviatn changed?
extern uint8_t freq_changed;                 // FREQ[012] changed?
extern uint8_t freq0val;                     // FREQ0 value
extern uint8_t freq1val;                     // FREQ1 value
extern uint8_t freq2val;                     // FREQ2 value

uint8_t AirMiniCV1;                          // The AirMini's address, HIGH uint8_t

uint8_t AirMiniCV17;                         // The AirMini's address, HIGH uint8_t
uint8_t AirMiniCV17tmp;                      // The AirMini's address, HIGH uint8_t,
                                             // temporary value until CV18 is reassigned in ops mode

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
// uint8_t  EECHANNELDefault;                 // Stored RF channel #

uint8_t  EEisSetlockedAntiphase = 3;        // Stored modem turn on/off is set
uint8_t  EElockedAntiphase = 4;             // Stored modem turn on/off
// uint8_t  EElockedAntiphaseDefault;          // Stored modem turn on/off

uint8_t  EEisSetdcLevel = 5;               // Stored DC output level is set
uint8_t  EEdcLevel = 6;                    // Stored DC output level if modem turned off
// uint8_t  EEdcLevelDefault;                 // Stored DC output level if modem turned off

uint8_t  EEisSetpowerLevel = 7;            // Stored DC output power level is set
uint8_t  EEpowerLevel = 8;                 // Stored DC output power level
// uint8_t  EEpowerLevelDefault;              // Stored DC output power level

uint8_t  EEisSetfilterModemData = 9;       // Stored filterModemData set flag
uint8_t  EEfilterModemData = 10;            // Stored filterModemData
// uint8_t  EEfilterModemDataDefault;          // Stored filterModemData

uint8_t  EEisSetAirMiniCV1 = 11;            // Stored AirMini decoder short address is set
uint8_t  EEAirMiniCV1 = 12;                 // Stored AirMini decoder short address
// uint8_t  EEAirMiniCV1Default;               // Stored AirMini decoder short address

uint8_t  EEisSetAirMiniCV17 = 13;           // Stored AirMini decoder high uint8_t address is set
uint8_t  EEAirMiniCV17 = 14;                // Stored AirMini decoder high uint8_t address
// uint8_t  EEAirMiniCV17Default;

uint8_t  EEisSetAirMiniCV18 = 15;           // Stored AirMini decoder low uint8_t address is set
uint8_t  EEAirMiniCV18 = 16;                // Stored AirMini decoder low uint8_t address
// uint8_t  EEAirMiniCV18Default;              // Stored AirMini decoder low uint8_t address

uint8_t  EEisSetAirMiniCV29 = 17;           // Stored AirMini decoder configuration variable is set
uint8_t  EEAirMiniCV29 = 18;                // Stored AirMini decoder configuration variable
// uint8_t  EEAirMiniCV29Default;              // Stored AirMini decoder configuration variable

#if defined(RECEIVER)
//{ RECEIVER
uint8_t  EEisSetInitialWaitPeriodSEC = 19;  // Stored AirMini decoder configuration variable
uint8_t  EEInitialWaitPeriodSEC = 20;       // Stored AirMini decoder configuration variable
// uint8_t  EEInitialWaitPeriodSECDefault;     // Stored AirMini decoder configuration variable
//} RECEIVER
#else
//{ TRANSMITTER
//} TRANSMITTER
#endif

///////////////////
// End: EEPROM data
///////////////////

enum {ACCEPTED, IGNORED, PENDING} CVStatus = ACCEPTED;

#if defined(USE_OLD_LCD) || defined(USE_NEW_LCD)
//{
uint8_t LCDAddress;                  // The I2C address of the LCD
bool LCDFound = false;               // Whether a valid lcd was found
#if defined(USE_OLD_LCD)
#define LCDCOLUMNS 16                // Number of LCD columns
#define LCDROWS 2                    // Number of LCD rows
#else
#define LCDCOLUMNS 128               // Number of LCD columns
// #define LCDROWS 32                   // Number of LCD rows
#define LCDROWS 64                   // Number of LCD rows
#endif
uint64_t LCDTimePeriod = 2ULL*SEC;   // Set up the LCD re-display time interval, 2 s
uint64_t LCDprevTime = 0;            // Initialize the last time displayed
bool LCDrefresh = false;             // Whether to refresh
char lcd_line[LCDCOLUMNS+1];         // Note the "+1" to insert an end null!
#if defined(USE_OLD_LCD)
LiquidCrystal_I2C lcd;               // Create the LCD object with a default address
#else
// Adafruit_SSD1306 lcd(LCDCOLUMNS, LCDROWS, &Wire, -1);
SSD1306AsciiAvrI2c lcd;
#endif
//}
#endif

// #define TURNOFFNOTIFYINTERRUPTS
#if defined(TURNOFFNOTIFYINTERRUPTS)
#pragma message "Turning off interrupts in notifyDccMsg"
#endif

///////////////////
// Start of code //
///////////////////
uint8_t notifyCVRead (uint16_t CV) {
    switch(CV) {
       case(1):
          return AirMiniCV1;
          break;
       case(17):
          return AirMiniCV17;
          break;
       case(18):
          return AirMiniCV18;
          break;
       case(29):
          return AirMiniCV29;
          break;
       case(255):
          return CHANNEL;
          break;
       case(254):
          return powerLevel;
          break;
       default:
          return (uint8_t)0;
          break;
    }
} // end of notifyCVRead

uint8_t notifyCVValid (uint16_t CV, uint8_t Writable) {
   return (uint8_t)1;
}

void notifyDccMsg(DCC_MSG * Msg) {
#if defined(TURNOFFNOTIFYINTERRUPTS)
  noInterrupts();  // Turning on/off interrupts does not seem to be needed
#endif
  if ((3 <= Msg->Size) && (Msg->Size <= 6)) {  // Check for a valid message
     msgIndexIn = (msgIndexIn+1) % MAXMSG;
     memcpy((void *)&msg[msgIndexIn], (void *)Msg, sizeof(DCC_MSG));
     dccptrIn = &msg[msgIndexIn];
     setScheduledTask(TASK1);            // Schedule the background task
     timeOfValidDCC = micros();          // Initialize the valid DCC data time
  }
#if defined(TURNOFFNOTIFYINTERRUPTS)
  interrupts();  // Turning on/off interrupts does not seem to be needed
#endif
#if defined(DEBUG)
  printMsgSerial();
#endif
}  // End of notifyDccMsg

uint8_t decoderInitialized = 0;

uint8_t notifyCVWrite (uint16_t CVnum, uint8_t CVval) {

   if (!decoderInitialized) return CVval;

   // Internal status defaults
   startModemFlag = 0;             // Initialize whether the modem will be restarted
   CVStatus = ACCEPTED;            // Set the default CV status

   switch (CVnum) {
      case  255:  // Set the channel number and reset related EEPROM values.
                  // Modest error checking. Verified this feature works
         if (CVval <= channels_max) {         // Check for good values
            checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, (uint8_t)CVval, 1);
            startModemFlag = 1;
         } else {                    // Ignore bad values
            CVStatus = IGNORED;
         }
      break;
      case  254:  // Set the RF power level and reset related EEPROM values.
                  // Verified this feature works.
         if (CVval <= 10) {
            checkSetDefaultEE(&powerLevel, &EEisSetpowerLevel,
                              &EEpowerLevel, (uint8_t)CVval, 1);  // Set powerLevel and reset
                                                                  // EEPROM values. Ignore bad values
            startModemFlag = 1;
         } else {
            CVStatus = IGNORED;
            CVval = powerLevel;
         }
      break;
      case  253:  // Turn off/on the modem for bad packet intervals and reset related EEPROM values.
                  // Verified this feature works
         checkSetDefaultEE(&lockedAntiphase, &EEisSetlockedAntiphase,
                           &EElockedAntiphase, (uint8_t)CVval, 1);  // Set lockedAntiphase
                                                                    // and reset EEPROM values
      break;
      case  252:  // Set the tooLong (in quarter second intervals) and reset related EEPROM values.
         tooLong = (uint64_t)CVval * QUARTERSEC;
                       break;
      case  248:  // Set the DC output level and reset related EEPROM values.
                  // Verified this feature works.
         checkSetDefaultEE((uint8_t *)&dcLevel, &EEisSetdcLevel, &EEdcLevel, (uint8_t)CVval, 1);
         // Set dcLevel and reset EEPROM values
      break;
      case  246:  // Set whether to always use modem data
         if (CVval) CVval = 1;  // Non-zero reset to 1
         checkSetDefaultEE(&filterModemData, &EEisSetfilterModemData,
                           &EEfilterModemData, (uint8_t)CVval, 1);  // Set filterModemData
                                                                    // and reset EEPROM values
      break;
#if defined(RECEIVER)
      case  245:  // Set the wait period in 1 second intervals
                  // - Nothing can be done with this until reset
         if (CVval <= 60)
            checkSetDefaultEE(&InitialWaitPeriodSEC, &EEisSetInitialWaitPeriodSEC,
                              &EEInitialWaitPeriodSEC,  (uint8_t)CVval, 1);  // Wait time in sec
         else
            CVStatus = IGNORED;
            CVval = InitialWaitPeriodSEC;
      break;
#endif

      case  243:  // Set the DEVIATN hex code
         deviatnval = CVval;
         deviatn_changed = 1;
         freq_changed |= 0b0001;
         freq_changed &= 0b0111;
         startModemFlag = 1;  // Reset the modem with a new deviatnval. Not persistent yet
      break;

#if defined(TRANSMITTER)
      case  242:  // Set the preamble counts
         preamble_bits = CVval;
      break;
#endif
      case  241:  // Set the timer long counts
         // CVval = (CVval > 0x43) ? (CVval):(0x43);  // Validation >= 95uS. Non-persistent
         timer_long = CVval;
      break;
      case  240:  // Set the timer short counts
         // Add validation
         timer_short = CVval;
      break;
      case  236:  // timer_long_cutout[3]
         // Add validation
        advance  = 2*(uint16_t)CVval;
      break;

      // CVs for resetting the FREQ
      case  235:  // Set the FREQ2 hex code
         if (freq_changed == 0b0111) { // freq1val, freq0val, deviatnval all set
            freq_changed |= 0b1000;
            freq2val = CVval;
            CHANNEL = 19;
            startModemFlag = 1;  // Now reset the frequency
         }
         else {
            if (freq_changed!=0b1000) CVStatus = IGNORED;
            else freq_changed = 0b0000;
         }
      break;
      case  234:  // Set the FREQ1 hex code
            freq_changed |= 0b0100;
            freq_changed &= 0b0111;
            freq1val = CVval;
      break;
      case  233:  // Set the FREQ0 hex code
            freq_changed |= 0b0010;
            freq_changed &= 0b0111;
            freq0val = CVval;
      break;

      case 29:    // Set the Configuration CV and reset related EEPROM values.
                  // Verified this feature works.
         checkSetDefaultEE(&AirMiniCV29, &EEisSetAirMiniCV29, &EEAirMiniCV29, (uint8_t)CVval, 1);
         AirMiniCV29Bit5 = AirMiniCV29 & 0b00100000;  // Save the bit 5 value of CV29
                                                      // (0: Short address, 1: Long address)
      break;
      case 18:    // Set the Long Address Low Byte CV and reset related EEPROM values.
                  // Verified this feature works.
                  // See NMRA S-9.2.1 Footnote 8.
         checkSetDefaultEE(&AirMiniCV17, &EEisSetAirMiniCV17, &EEAirMiniCV17,
                           (uint8_t)AirMiniCV17tmp, 1);
         checkSetDefaultEE(&AirMiniCV18, &EEisSetAirMiniCV18, &EEAirMiniCV18, (uint8_t)CVval, 1);
      break;
      case 17:    // Set the Long Address High Byte CV and save values after validation (do NOT
                  // write to AirMini's CV17 or EEPROM yet!).
         if ((0b11000000 <= CVval) && (CVval <= 0b11100111)) {  // NMRA standard 9.2.2,
                                                                // Paragraphs 129-135, footnote 8
            AirMiniCV17tmp = CVval;    // Do not take effect until CV18 is written!
                                       // NMRA Standard 9.2.1, footnote 8.
            CVStatus = PENDING;
         } else {
            CVStatus = IGNORED;
         }
      break;
      case  8:  // Full EEPROM Reset and reboot!
         if (CVval == 8) {
#if defined(USE_OLD_LCD) || defined(USE_NEW_LCD)
//{
            if (LCDFound) {

               snprintf(lcd_line, sizeof(lcd_line), "Keep Power ON!");

               lcd.clear();
#if defined(USE_OLD_LCD)
               lcd.setCursor(0, 0);  // column, row
#endif
               LCD_PRINT(lcd_line);
               snprintf(lcd_line, sizeof(lcd_line), "Factory Reset...");
#if defined(USE_OLD_LCD)
               lcd.setCursor(0, 1);  // column, row
#endif
               LCD_PRINT(lcd_line);
            }
//}
#endif
            eepromClear();
            reboot();  // No need for sei, you're starting over...
         } else {
            CVStatus = IGNORED;
         }
      break;
      case 1:     // Set the Short Address CV and reset related EEPROM values after validation.
                  // Verified this feature works.
         if ((0 < CVval) && (CVval < 128)) {  // CV1 cannot be outside this range.
                                              // Some decoders limit 0<CVval<100
            checkSetDefaultEE(&AirMiniCV1, &EEisSetAirMiniCV1, &EEAirMiniCV1, (uint8_t)CVval, 1);
         } else {
            CVStatus = IGNORED;
         }
      break;
      default:
         CVStatus = IGNORED;
      break;
   }  // end of switch (CVnum)

#if defined(USE_OLD_LCD) || defined(USE_NEW_LCD)
   if (LCDFound) LCD_CVval_Status(CVnum, CVval, CVStatus);
#endif
   if (startModemFlag) {
      strobeSPI(SIDLE);         // Stop the modem
      startModem(CHANNEL, MODE);  // Restart on possible-new Airwire Channel and mode
                                  // or power level
   }
   return CVval;
} // end of notifyCVWrite

// Output timer
// Setup Timer1.
// Configures the 16-Bit Timer1 to generate an interrupt at the specified frequency.
// Returns the time load value which must be loaded into TCNT1 inside your ISR routine.
void SetupTimer1() {
  TCCR1A = 0;
  TCCR1B = 0 << CS12 | 0 << CS11 | 1 << CS10;  // No prescaling

  // Timer1 Overflow Interrupt Enable
  TIMSK1 = 1 << TOIE1;

  // load the timer for its first cycle
  TCNT1 = timer_short;
}  // End of SetupTimer1

#pragma GCC push_options
#pragma GCC optimize("-O3")
// Timer1 overflow interrupt vector handler
ISR(TIMER1_OVF_vect) {
  // Capture the current timer value TCNT1. This is how much error we have
  // due to interrupt latency and the work in this function
  // Reload the timer and correct for latency.
  // for more info, see http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/

  while (TCNT1 < advance) sleep();  // A short delay for latency leveling
#if defined(PRINT_LATENCY)
  latency = TCNT1;
#endif

  // for every second interupt just toggle signal
  if (every_second_isr)  {

     if (useModemData) {      // If not using modem data, the level is set elsewhere
        OUTPUT_HIGH;  // Output high
#if defined(TRANSMITTER)
        if ((preamble_bits >= MAXIMUM_PREAMBLE_BITS) && (current_state == CUTOUT)) {
           if (num_cutout==1) {
              timer_val = timer_long;
           }
        }
#endif
     }

     every_second_isr = 0;
  }  else  {  // != every second interrupt, advance bit or state

     if (useModemData)        // If not using modem data, the level is set elsewhere
        OUTPUT_LOW;  // Output low

     every_second_isr = 1;
     current_state = next_state;

     switch (current_state)  {
        case CUTOUT:
           timer_val = timer_short;  // second half will be reset
           num_cutout++;
           // get next message
           if (msgIndexOut != msgIndexIn) {
              msgIndexOut = (msgIndexOut+1) % MAXMSG;
              dccptrISR = &msg[msgIndexOut];
              dccptrOut = dccptrISR;  // For display only
              useModemData = 1;
              next_state = PREAMBLE;  // jump out of state
           } // else, repeat the pulse or turn on DC output
           else if (num_cutout >= MAX_NUM_CUTOUT) { // Set up filtering for either DC or preamble bits
              num_cutout = 2; // restart the counter, but prevent long pulse cutout
           }
           if (next_state == PREAMBLE) {
#if defined(TRANSMITTER)
              if (preamble_bits >= MINIMUM_PREAMBLE_BITS)  // Ensure meets a reasonable minimum
                 preamble_count = preamble_bits;  // large enough to satisfy Airwire!
              else
                 preamble_count = dccptrISR->PreambleBits; // use what was given
#else
              preamble_count = dccptrISR->PreambleBits;
#endif
           }
        break;
        case PREAMBLE:
           timer_val = timer_short;
           preamble_count--;
           if (preamble_count == 0)  {  // advance to next state
             next_state = STARTBYTE;  // Really STARTPACKET
             byteIndex = 0;  // start msg with uint8_t 0
           }
        break;
        case STARTBYTE:
           timer_val = timer_long;
           // then advance to next state
           next_state = SENDBYTE;
           // goto next uint8_t ...
           cbit = 0x80;  // send this bit next time first
           outbyte = dccptrISR->Data[byteIndex];
        break;
        case SENDBYTE:
           timer_val = (outbyte & cbit) ? timer_short : timer_long;
           cbit = cbit >> 1;
           if (cbit == 0)  {  // last bit sent, is there a next uint8_t?
              byteIndex++;
              if (byteIndex >= dccptrISR->Size)  {
                // this was already the XOR uint8_t then advance to stop packet! NOT preamble
                next_state = STOPPACKET;  // NOT preamble!!!
              }  else  {
                // send separtor and advance to next uint8_t
                next_state = STARTBYTE;
              }
           }
        break;
        case STOPPACKET:
           timer_val = timer_short;
           next_state = CUTOUT;
           num_cutout = 0;
        break;
     }  // end of switch
  }  // end of else ! every_seocnd_isr

  TCNT1 += timer_val;
}  // End of ISR
#pragma GCC pop_options

#if defined(DEBUG)
void printMsgSerial() {
  Serial.print("msg[");
  Serial.print(msgIndexIn, HEX);
  Serial.print("]:\n");
  Serial.print(" len: ");
  Serial.print(msg[msgIndexIn].Size, HEX);
  Serial.print("\n");
  for (uint8_t i = 0; i < msg[msgIndexIn].Size; i++) {
     Serial.print(" data[");
     Serial.print(i, HEX);
     Serial.print("]: ");
     Serial.print(msg[msgIndexIn].Data[i], HEX);
     Serial.print("\n");
  }
}
#endif

// Function to combine High and Low bytes into 16 uint8_t variable
uint16_t combineHighLow(uint8_t High, uint8_t Low) {
  uint16_t tmp16 = (uint16_t)High;
  tmp16 <<= 8;
  uint16_t tmpLow16 = (uint16_t)Low;
  tmp16 |= tmpLow16;
  return (tmp16);
}

void reboot() {
  cli();                     // Ensure that when setup() is called, interrupts are OFF
  asm volatile ("  jmp 0");  // "Dirty" method because it simply restarts the SW, and does NOT reset the HW
}

void eepromClear() {
  for (unsigned int i = 0 ; i < EEPROM.length() ; i++) {
     EEPROM.write(i, 0);
  }
}

/* Not used
void checkSetDefault(uint8_t *TargetPtr, const uint8_t *EEisSetTargetPtr, 
                    const uint8_t *EETargetPtr, uint8_t defaultValue, uint8_t forceDefault) {
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
*/

// Function, based on the value of forceDefault:
//    - TRUE:   TargetPtr's value and its related EEPROM variables are forced to use defaultValue
//    - FALSE:  extract and use EEPROM data, if previously-set, to set the TargetPtr's value
void checkSetDefaultEE(uint8_t *TargetPtr, const uint8_t *EEisSetTargetPtr,
                       const uint8_t *EETargetPtr, uint8_t defaultValue, uint8_t forceDefault) {
  uint8_t isSet;
#if defined(DEBUG)
  uint8_t isSet_Save;
#endif
  eeprom_busy_wait();
  isSet = (uint8_t)eeprom_read_byte((const uint8_t *)EEisSetTargetPtr);
  if ((isSet != ISSET) || forceDefault) {
#if defined(DEBUG)
     isSet_Save = isSet;
#endif
     *TargetPtr = defaultValue;
     eeprom_busy_wait();

     eeprom_update_byte((uint8_t *)EEisSetTargetPtr, (const uint8_t)ISSET);
     delay(EEPROMDELAYMS);  // Magic delay time to ensure update is complete
     eeprom_busy_wait();
     for (uint8_t i = 0; i < 10; i++) {
        isSet = (uint8_t)eeprom_read_byte((const uint8_t *)EEisSetTargetPtr);
        if (isSet != ISSET) {
#if defined(DEBUG)
           Serial.print("ISSET error: isSet, ISSET = ");
           Serial.print(isSet);
           Serial.print(", ");
           Serial.print(ISSET);
           Serial.print("\n");
           delay(10);
#endif
           eeprom_update_byte((uint8_t *)EEisSetTargetPtr, (const uint8_t)ISSET);
           eeprom_busy_wait();
        } else {
            break;
        }
     }

     eeprom_update_byte((uint8_t *)EETargetPtr, defaultValue);
     delay(EEPROMDELAYMS);  // Magic delay time to ensure update is complete
     eeprom_busy_wait();

     for (uint8_t i = 0; i < 10; i++) {
        *TargetPtr = (uint8_t)eeprom_read_byte((const uint8_t *)EETargetPtr);
        if (*TargetPtr != defaultValue) {
#if defined(DEBUG)
           Serial.print("TargetPtr error: *TgtPtr, defaultValue = ");
           Serial.print(*TargetPtr);
           Serial.print(", ");
           Serial.print(defaultValue);
           Serial.print("\n");
           delay(10);
#endif
           eeprom_update_byte((uint8_t *)EETargetPtr, defaultValue);
           eeprom_busy_wait();
        } else {
           break;
        }
     }

#if defined(DEBUG)
     Serial.print("   Reset: *TargetPtr, *EEisSetTargetPtr, *EETargetPtr, ISSET, isSet, forceDefault, "
                  "defaultValue (Orig isSet): ");
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
  } else {
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
}  // end of checkSetDefaultEE

#if defined(USE_OLD_LCD) || defined(USE_NEW_LCD)
//{  // USE_OLD_LCD
void LCD_Banner() {
#if defined(USE_OLD_LCD)
  lcd.setCursor(0, 0);              // Set initial column, row
#else
  lcd.clear();              // Set initial column, row
#endif
  if (whichBanner == INITIAL)
     LCD_PRINT(bannerString);   // Banner
  else
     LCD_PRINT("ProMini Air Info");
#if defined(USE_OLD_LCD)
  lcd.setCursor(0, 1);              // Set next line column, row
#endif
  LCD_PRINT("H:" HWVERSION " S:" SWVERSION "/" FREQMH "MH");   // Show state
  LCDprevTime  = micros();     // Set up the previous display time
  LCDrefresh = true;
}  // end of LCD_Banner

void LCD_Addr_Ch_PL() {
  if (!printIn) dccptrTmp = dccptrOut;
  else dccptrTmp = dccptrIn;

  lcd.clear();
#if defined(USE_OLD_LCD)
  lcd.setCursor(0, 0);  // column, row
#endif
  if (printDCC) {
     if (dccptrTmp->Data[0] < 128) {
           snprintf(lcd_line, sizeof(lcd_line), "Msg Ad: %d(S)", (uint16_t)dccptrTmp->Data[0]);
     } else if (dccptrTmp->Data[0] >= 192) {
           uint16_t TargetAddress = (dccptrTmp->Data[0]-192)<<8 | dccptrTmp->Data[1];
           snprintf(lcd_line, sizeof(lcd_line), "Msg Ad: %d(L)", TargetAddress);

     } else {
        // Accessory address decoding
        uint8_t tmpuint8;
        uint16_t TargetAddress = (dccptrTmp->Data[0] & 0b00111111); // Lower 6 bits, 0b00000000 00(A5)(A4)(A3)(A2)(A1)(A0)
        if (dccptrTmp->Data[1] & 0b10000000) {
           tmpuint8 = ~dccptrTmp->Data[1] & 0b01110000; // Ones complement upper 3 bits 0b0(A8)(A7)(A6)0000
           TargetAddress = (((((tmpuint8 << 2) | TargetAddress) - 1) << 2) | ((dccptrTmp->Data[1] & 0b00000110) >> 1)) + 1;
        } else {
           tmpuint8  = (dccptrTmp->Data[1] & 0b00000110) >> 1; // 0b0000(A7)(A6)0 -> 0b000000(A7)(A6)
           tmpuint8 |= (dccptrTmp->Data[1] & 0b01110000) >> 2; // 0b00000(A7)(A6) | 0b0(A10)(A9)(A8)0000 -> 0b000(A10)(A9)(A8)(A7)(A6)
           // 0b00000000 00(A5)(A4)(A3)(A2)(A1)(A0) | 0b00000(A10)(A9)(A8) (A7)(A6)000000 -> 0b00000(A10)(A9)(A8) (A7)(A6)(A5)(A4)(A3)(A2)(A1)(A0)
           TargetAddress |= ((uint16_t)tmpuint8 << 6); 
        }
        snprintf(lcd_line, sizeof(lcd_line), "Msg Ad: %d(A)", TargetAddress);
     }
  } else {
     uint16_t AirMiniAddress_int = DCC.getAddr();
        if (AirMiniCV29Bit5) 
           snprintf(lcd_line, sizeof(lcd_line), "My Ad: %d(L)", AirMiniAddress_int);
        else
           snprintf(lcd_line, sizeof(lcd_line), "My Ad: %d(S)", AirMiniAddress_int);
  }

  LCD_PRINT(lcd_line);
#if defined(USE_OLD_LCD)
  lcd.setCursor(0, 1);  // column, row
#endif

  if (printDCC) {
     snprintf(lcd_line, sizeof(lcd_line), "                ");
     printDCC = 0;
     lcd_line[0] = 'P';
     lcd_line[1] = 'M';
     lcd_line[2] = 'A';
     if (printIn) {
        lcd_line[3] = '<';
        printIn = false;
     } else {
        lcd_line[3] = '>';
        printIn = true;
     }
     for (uint8_t i = 0; i < dccptrTmp->Size; i++) {
        snprintf(&lcd_line[2*i+4], 3, "%02X", dccptrTmp->Data[i]);
     }
     // snprintf(&lcd_line[4],5,"P:%02d", dccptrTmp->PreambleBits);  // For debugging only!!
#if defined(PRINT_LATENCY)
     snprintf(&lcd_line[4], 8, "L:%05d", latency);  // For debugging only!!
#endif
  } else {
     printDCC = 1;
#if defined(NAEU_900MHz)
     if (CHANNEL <= channels_na_max)
        regionNum = 0;
     else
        regionNum = 1;
#endif

#if defined(TRANSMITTER)
     snprintf(lcd_line, sizeof(lcd_line), "Ch:%d(%s) PL:%d", CHANNEL, regionString[regionNum], powerLevel);
#else
     snprintf(lcd_line, sizeof(lcd_line), "Ch:%d(%s) Filt:%d", CHANNEL, regionString[regionNum], filterModemData);
#endif
  }

  LCD_PRINT(lcd_line);

  return;
}  // end of LCD_Addr_Ch_PL

void LCD_CVval_Status(uint8_t CVnum, uint8_t CVval, uint8_t CVStatus) {

  lcd.clear();

#if defined(USE_OLD_LCD)
  lcd.setCursor(0, 0);  // column, row
#endif
  switch (CVStatus) {
     case ACCEPTED:
        snprintf(lcd_line, sizeof(lcd_line), "Changed:");
     break;
     case IGNORED:
        snprintf(lcd_line, sizeof(lcd_line), "Ignored:");
     break;
     case PENDING:
        snprintf(lcd_line, sizeof(lcd_line), "Pending:");
     break;
  }
  LCD_PRINT(lcd_line);

#if defined(USE_OLD_LCD)
  lcd.setCursor(0, 1);  // column, row
#endif
  snprintf(lcd_line, sizeof(lcd_line), "CV%d=%d", CVnum, CVval);
  LCD_PRINT(lcd_line);

  LCDprevTime  = micros();
  LCDrefresh = true;

  return;
}  // end of LCD_CVval_Status

void LCD_Wait_Period_Over(uint16_t status) {

  lcd.clear();

#if defined(USE_OLD_LCD)
  lcd.setCursor(0, 0);  // column, row
#endif
  if (!status) {
     snprintf(lcd_line, sizeof(lcd_line), "NO valid");
  } else {
     snprintf(lcd_line, sizeof(lcd_line), "Found valid");
  }
  LCD_PRINT(lcd_line);

#if defined(USE_OLD_LCD)
  lcd.setCursor(0, 1);  // column, row
#endif

#if defined(NAEU_900MHz)
  if (CHANNEL <= channels_na_max)
    regionNum = 0;
  else
    regionNum = 1;
#endif

  snprintf(lcd_line, sizeof(lcd_line), "RF on Ch: %d(%s)", CHANNEL, regionString[regionNum]);
  LCD_PRINT(lcd_line);
  LCDprevTime  = micros();
  LCDrefresh = true;

  return;
}  // end of LCD_Wait_Period_Over
//}  // USE_OLD_LCD
#endif

void setup() {
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
  if (SET_DEFAULT != ISSET)
     SET_DEFAULT = 1;
  else
     SET_DEFAULT = 0;
  eeprom_busy_wait();
#if defined(DEBUG)
  Serial.print("Final SET_DEFAULT: "); Serial.print(SET_DEFAULT); Serial.print("\n");
  delay(100);
#endif

  // Get the CHANNEL # stored in EEPROM and validate it
  // eeprom_update_byte(&EECHANNELDefault, CHANNELDEFAULT );
  checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL,
                    (uint8_t)CHANNELDEFAULT, SET_DEFAULT);      // Validate the channel, it's possible
                                                                // the EEPROM has bad data
  if (CHANNEL > channels_max)
     checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL,
                       (uint8_t)CHANNELDEFAULT, 1);  // Force the EEPROM data to use CHANNEL 0,
                                                     // if the CHANNEL is invalid

  // Just flat out set the powerLevel
  // eeprom_update_byte(&EEpowerLevelDefault, POWERLEVELDEFAULT );
  checkSetDefaultEE(&powerLevel, &EEisSetpowerLevel, &EEpowerLevel,
                    (uint8_t)POWERLEVELDEFAULT, SET_DEFAULT);  // Use EEPROM value if it's been set,
                                                               // otherwise set to 1 and set EEPROM values

  // Set the alternate DC output level to HIGH or LOW (i.e., bad CC1101 data)
  // The level of this output can be used by some decoders. The default is HIGH.
  // eeprom_update_byte(&EEdcLevelDefault, DCLEVELDEFAULT );
  checkSetDefaultEE((uint8_t *)&dcLevel, &EEisSetdcLevel, &EEdcLevel,
                    (uint8_t)DCLEVELDEFAULT, SET_DEFAULT);  // Use EEPROM value if it's been set, otherwise set
                                                               // to 1 and set EEPROM values
  // Turn the lockedAntiphase OFF/ON option.
  // eeprom_update_byte(&EElockedAntiphaseDefault, LOCKEDANTIPHASEDEFAULT );
  checkSetDefaultEE(&lockedAntiphase, &EEisSetlockedAntiphase, &EElockedAntiphase,
                    (uint8_t)LOCKEDANTIPHASEDEFAULT, SET_DEFAULT);  // Use EEPROM value if it's been set,
                                                                    // otherwise set to 1 and set EEPROM values

  // Set whether to always use modem data on transmit
  // eeprom_update_byte(&EEfilterModemDataDefault, FILTERMODEMDATADEFAULT );
  checkSetDefaultEE(&filterModemData, &EEisSetfilterModemData, &EEfilterModemData,
                    (uint8_t)FILTERMODEMDATADEFAULT, SET_DEFAULT);  // Use EEPROM value if it's been set,
                                                                    // otherwise set to 0

  // Get up addressing-related CV's from EEPROM, or if not set set them in EEPROM
  // eeprom_update_byte(&EEAirMiniCV1Default, AIRMINICV1DEFAULT );
  checkSetDefaultEE(&AirMiniCV1,  &EEisSetAirMiniCV1,  &EEAirMiniCV1,
                    (uint8_t)AIRMINICV1DEFAULT, SET_DEFAULT);  // Short address. By default, not using

  // eeprom_update_byte(&EEAirMiniCV17Default, AIRMINICV17DEFAULT );
  checkSetDefaultEE(&AirMiniCV17, &EEisSetAirMiniCV17, &EEAirMiniCV17,
                    (uint8_t)AIRMINICV17DEFAULT, SET_DEFAULT);  // High uint8_t to set final address to 9000
  AirMiniCV17tmp = AirMiniCV17;                                 // Due to the special nature of CV17 paired with CV18

  // eeprom_update_byte(&EEAirMiniCV18Default, AIRMINICV18DEFAULT );
  checkSetDefaultEE(&AirMiniCV18, &EEisSetAirMiniCV18, &EEAirMiniCV18,
                    (uint8_t)AIRMINICV18DEFAULT, SET_DEFAULT);  // Low uint8_t to set final address to
                                                                // 9000/9001 for transmitter/receiver

  // eeprom_update_byte(&EEAirMiniCV29Default, AIRMINICV29DEFAULT );
  checkSetDefaultEE(&AirMiniCV29, &EEisSetAirMiniCV29, &EEAirMiniCV29,
                    (uint8_t)AIRMINICV29DEFAULT, SET_DEFAULT);  // Set CV29 so that it will use a long address
  AirMiniCV29Bit5 = AirMiniCV29 & 0b00100000;  // Save the bit 5 value of CV29 (0: Short address, 1: Long address)

#if defined(RECEIVER)
//{ RECEIVER
  // eeprom_update_byte(&EEInitialWaitPeriodSECDefault, INITIALWAITPERIODSECDEFAULT );
  checkSetDefaultEE(&InitialWaitPeriodSEC, &EEisSetInitialWaitPeriodSEC, &EEInitialWaitPeriodSEC,
                    (uint8_t)INITIALWAITPERIODSECDEFAULT, SET_DEFAULT);  // Wait time in sec
//} RECEIVER
#else
//{ TRANSMITTER
//} TRANSMITTER
#endif

  // Now set to not first time
  eeprom_update_byte((uint8_t *)&EEFirst, (const uint8_t)ISSET);
  eeprom_busy_wait();

  /////////////////////////////////////////////////////
  // End: Let's get the slow EEPROM stuff done first //
  /////////////////////////////////////////////////////


  /////////////////////////////////
  // Initialization of variables //
  /////////////////////////////////

  startModemFlag = 0;                          // Initialize the start-modem flag
  useModemData = 1;                            // Initialize use-of-modem-data state

  memset(dccptrNULL, 0, sizeof(dccptrNULL));                      // Create a null dccptrIn for CV setting

  dccptrIn =  (volatile DCC_MSG *)&msgIdle;  // Well, set it to something.
  dccptrOut = (volatile DCC_MSG *)&msgIdle;  // Well, set it to something.
  dccptrISR = (volatile DCC_MSG *)&msgIdle;  // Well, set it to something.

  ///////////////////////////////////////////////
  // Set up the hardware and related variables //
  ///////////////////////////////////////////////

  pinMode(OUTPUT_ENABLE, OUTPUT);             //
  digitalWrite(OUTPUT_ENABLE, 1);             // Now used for Output Enable

  pinMode(DCC_DIAG1, OUTPUT);                 //
  digitalWrite(DCC_DIAG1, 0);                 // Will use this for diagnostics

  // Experimental: Initially clear TASK1
  clearScheduledTask(TASK1);

  // Set up the input and output pins

  DCC.pin(EXTINT_NUM, INPUT_PIN, 1);  // register External Interrupt # and Input Pin of input source. Enable pullup!
                                      // Important. Pins and interrupt #'s are correlated.

  SET_OUTPUTPIN;

#if defined(RECEIVER)
  OUTPUT_OFF;
#endif

  // Final timing-sensitive set-ups

  // Set up slow-time variables
  then = micros();                            // Grab Current Clock value for the loop below

#if defined(USE_OLD_LCD) || defined(USE_NEW_LCD)
  // Scan for I2C devices
  Wire.begin();  // Wire communication begin
  uint8_t nDevices = 0;
  uint8_t address;
  uint8_t error;
  for (address = 1; address < 127; address++) {
     // The i2c_scanner uses the return value of
     // the Write.endTransmisstion to see if
     // a device did acknowledge to the address.
     Wire.beginTransmission(address);
     error = Wire.endTransmission();

     if (error == 0) {
        nDevices++;
        LCDAddress = address;
     }
  }

  if (nDevices == 1) {
     LCDFound = true;
     LCDrefresh = true;
#if defined(USE_OLD_LCD)
     lcd.init(LCDAddress, LCDCOLUMNS, LCDROWS);    // Initialize the LCD
     lcd.backlight();                            // Backlight it
#else
     // lcd.begin(&Adafruit128x64, LCDAddress);
     lcd.begin(&Adafruit128x32, LCDAddress);
     lcd.setFont(Adafruit5x7);
#endif
     whichBanner = INITIAL;
     LCDprevTime = micros()+LCDTimePeriod;
  } else {
     LCDFound = false;
     LCDrefresh = false;
  }
#endif

  // Call the main DCC Init function to enable the DCC Receiver
  decoderInitialized = 0;
  DCC.init(MAN_ID_DIY, 10,   FLAGS_MY_ADDRESS_ONLY, 0);
  decoderInitialized = 1;
  timeOfValidDCC = micros();

#if defined(RECEIVER)
  initialWait = 1;
  startInitialWaitTime = timeOfValidDCC;      // Initialize the start of the wait time (never is not a good value)
  endInitialWaitTime =
    startInitialWaitTime + (uint64_t)InitialWaitPeriodSEC * SEC;  // Initialize the end of the wait time
#else
  initialWait = 0;
#endif

  // Start the coms with the modem
  initializeSPI();                            // Initialize the SPI interface to the radio
  delay(10);                                  // Wait a bit for the SPI
  startModem(CHANNEL, MODE);                  // Start radio on this Channel

  // Set up the waveform generator timer
  SetupTimer1();  // Set up interrupt Timer 1

  sei();                                      // enable interrupts
}  // End of setup

void loop() {
  /* Check High Priority Tasks First */
  DCC.process();  // The DCC library does it all with the callback notifyDccMsg!

  switch (masterSchedule()) {
     case TASK0:                      // Highest Priority Task goes here
        clearScheduledTask(TASK0);
     break;
     case TASK1:                      // Just pick a priority for the DCC packet, TASK1 will do
        // dccptrIn = getDCC();       // we are here, so a packet has been assembled, get a pointer to our DCC data

        //
        // Special processing for AirMini OPS mode //
        //
        // Moved to notifyCVWrite
        //
        // End ofSpecial processing for AirMini OPS mode //
        //

        clearScheduledTask(TASK1);      // all done, come back next time we are needed

     break;  // TASK1 break
  }  // End of switch (masterSchedule())

  /**** After checking highest priority stuff, check for the timed tasks ****/

  now = micros();


  if ((now - then) > BACKGROUNDTIME) {          // Check for Time Scheduled Tasks
                                                 // A priority Schedule could be implemented in here if needed
     then = micros();             // Grab Clock Value for next time

#if defined(USE_OLD_LCD) || defined(USE_NEW_LCD)
//{
     if (LCDrefresh && ((then-LCDprevTime) >= LCDTimePeriod)) {
        if (whichBanner == NONE) {
           LCD_Addr_Ch_PL();           // Update the display of address, chanel #, and power level
        } else {
           if (!initialWait) {
              if (whichBanner == INITIAL) {
                 LCD_Banner();
                 whichBanner = INFO;
              } else {
                 LCD_Banner();
                 whichBanner = NONE;
              }
           }
        }
        LCDprevTime = then;              // Slowly... at 1 sec intervals
        LCDrefresh = true;
     }
//}
#endif

#if defined(TRANSMITTER)
//{ TRANSMITTER
     strobeSPI(MODE);         // keep the radio awake in MODE
//} TRANSMITTER
#else
//{ RECEIVER
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
     //                                  reset the DC level by accessing the AirMini on 9900 and set CV248
     //                                  to 0 (LOW) or non-zero (HIGH).
     // It is ESSENTIAL that you do NOT jump to DC output too soon! Otherwise Airwire programming will NOT
     // work well. Outputting an indefinite number of preamble bits seems to work OK with Airwire programming.
     // Also, do NOT jump to DC output until the receiver has set its final channel.
     if (filterModemData & (!initialWait) && ((then-timeOfValidDCC) >= tooLong)) {
         useModemData = 0; // false use-of-modem-data state
     } else {
         useModemData = 1;
     }
     if (!useModemData) {  // If not using modem data, ensure the output is set to a DC level
         if (dcLevel)
            OUTPUT_HIGH;  // HIGH
         else
            OUTPUT_LOW;   // LOW
     }

     // Special processing for channel search
     if (initialWait) {
        // If we received a valid DCC signal during the intial wait period, stop waiting and proceed normally
        if (timeOfValidDCC > startInitialWaitTime) {
           initialWait = 0;
#if defined(USE_OLD_LCD) || defined(USE_NEW_LCD)
           if (LCDFound) LCD_Wait_Period_Over(1);
#endif
        } else {  // Othewise, continue to wait
           // If it's too long, then reset the modem to the next channel
           if (then > endInitialWaitTime) {
#if defined(USE_OLD_LCD) || defined(USE_NEW_LCD)
              if (LCDFound) LCD_Wait_Period_Over(0);
#endif
              if (++searchChannelIndex <= sizeof(searchChannels)) {
              // Keep searching...
                 // Update the seach channel and endInitialWaitTime
                 CHANNEL = searchChannels[searchChannelIndex-1];
                 // Re-initialize the start of the wait time
                 startInitialWaitTime = then;
                 // Re-initialize the end of the wait time
                 endInitialWaitTime =
                     startInitialWaitTime
                   + (uint64_t)InitialWaitPeriodSEC * SEC;
              } else {
              // Last resort
                 initialWait = 0;
                 CHANNEL = CHANNELDEFAULT;    // Reset to the last resort channel
              }

              // Stop the modem
              strobeSPI(SIDLE);
              // Call the main DCC Init function to enable the DCC Receiver
              // DCC.init(MAN_ID_DIY, 100,   FLAGS_DCC_ACCESSORY_DECODER, 0 );
              // Restart on Airwire selected and mode (or power level)
              startModem(CHANNEL, MODE);
           }  // end of wait time over
        }  // end of continue to wait
     }  // End of special processing for channel search
//} RECEIVER
#endif
  }  // end of if ((now - then) > BACKGROUNDTIME )
}  // end of loop

////////////////////////////////////////////////
// End of file AirMiniSketchTransmitter_Nmra.ino
////////////////////////////////////////////////
