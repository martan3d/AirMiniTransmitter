#define TRANSMIT

#include <dcc.h>
#include <spi.h>
#include <uart.h>
#include <schedule.h>
#include <servotimer.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <string.h>

#undef DCCLibrary
#define DCCLibrary

#ifdef DCCLibrary
///////////////////
///////////////////
///////////////////
#include <DCCLibrary.h>
// Externs
extern bool (*GetNextMessage)(void); // For DCCLibrary
// For DCCLibrary
#ifdef TRANSMIT
#define DCC_PIN    2     // Arduino pin for DCC out. This pin is connected to "DIRECTION" of LMD18200
#else
#define DCC_PIN    4     // Arduino pin for DCC out. This pin is connected to "DIRECTION" of LMD18200
#endif
#define DCC_DIAG0  5     // Diagnostic Pin #1
#define DCC_DIAG1  6     // Diagnostic Pin #2
#define msgSize 32       // The size of the ring buffer. Per Martin's new code
// Implement a ring buffer
volatile Message msg[msgSize] = {      // -> to DCCLibrary.c
    { { 0xFF, 0, 0xFF, 0, 0, 0, 0}, 3},
    { { 0xFF, 0, 0xFF, 0, 0, 0, 0}, 3},
    { { 0, 0, 0, 0, 0, 0, 0}, 0}
  };      
// Private msg sent to DCCLibrary.c ISR
volatile Message msgExtracted[2] = {   // -> to DCCLibrary.c
    { { 0xFF, 0, 0xFF, 0, 0, 0, 0}, 3}, // Will be overwritten in NextMessage
    { { 0xFF, 0, 0xFF, 0, 0, 0, 0}, 3}, // The idle packet we will send in DCCLibrary.c when conditions permit/necessitate
};

volatile uint8_t lastMessageInserted  = 1;
volatile uint8_t lastMessageExtracted = 0;
volatile uint8_t currentIndex = 0;   // -> to DCCLibrary.c
uint8_t          newIndex     = 2;

// For opMode debug
uint64_t opModeTime = 0; 
uint64_t opModeDuration = 4000000; // 1 s

///////////////////
///////////////////
///////////////////
#endif


// Times
#define MILLISEC          4000   // @16Mhz, this is 1ms, 0.001s
#define QUARTERSEC     1000000   // @16Mhz, this is 0.25s
#define BACKGROUNDTIME   32000   // @16Mhz, this is 8ms

// CC1101 codes
#define RXMODE  0x34             // C1101 modem RX mode
#define RX      0x34
#define TX      0x35
#define STOP    0x36

// Need to test
#define DOUBLE_PASS 1            // Do a double pass on CV setting (currently does not work)

int64_t now;
int64_t then;

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
volatile uint8_t useModemData = 0;           // Initial setting for use-of-modem-data state
int64_t idlePeriod = 0;                      // 0 msec, changed to variable that might be changed by SW
uint8_t idlePeriodms = 0;                    // 0 msec, changed to variable that might be changed by SW
int64_t lastIdleTime = 0;
int64_t tooLong  = 4000000;                  // 1 sec, changed to variable that might be changed by SW
int64_t sleepTime = 8000000;                 // 2 sec, changed to variable that might be changed by SW
int64_t timeOfValidDCC;                      // Time stamp of the last valid DCC packet
int64_t inactiveStartTime;                   // Time stamp if when modem data is ignored
uint16_t maxTransitionCount;                 // Maximum number of bad transitions to tolerate before ignoring modem data
uint8_t maxTransitionCountLowByte=100;       // High byte of maxTransitionCount
uint8_t maxTransitionCountHighByte=0;        // Low byte of maxTransitionCount
#define LED_ON 255                           // Analog PWM ON (full on)
#define LED_OFF 0                            // Analog PWM OFF (full off)


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

// EEPROM data for persistence after turn-off of the AirMini
uint8_t  EEMEM EEisSetCHANNEL;               // Stored RF channel is set
uint8_t  EEMEM EECHANNEL;                    // Stored RF channel #
uint8_t  EEMEM EECHANNELDefault;             // Stored RF channel #

uint8_t  EEMEM EEisSetturnModemOnOff;        // Stored modem turn on/off is set
uint8_t  EEMEM EEturnModemOnOff;             // Stored modem turn on/off
uint8_t  EEMEM EEturnModemOnOffDefault;      // Stored modem turn on/off

uint8_t  EEMEM EEisSetdcLevel;               // Stored DC output level is set
uint8_t  EEMEM EEdcLevel;                    // Stored DC output level if modem turned off
uint8_t  EEMEM EEdcLevelDefault;             // Stored DC output level if modem turned off

uint8_t  EEMEM EEisSetpowerLevel;            // Stored DC output power level is set
uint8_t  EEMEM EEpowerLevel;                 // Stored DC output power level 
uint8_t  EEMEM EEpowerLevelDefault;          // Stored DC output power level 

uint8_t  EEMEM EEisSetidlePeriodms;          // Stored idlePeriodms set flag
uint8_t  EEMEM EEidlePeriodms;               // Stored idlePeriod in ms
uint8_t  EEMEM EEidlePeriodmsDefault;        // Stored idlePeriod in ms

uint8_t  EEMEM EEisSetAirMiniCV1;            // Stored AirMini decoder short address is set
uint8_t  EEMEM EEAirMiniCV1;                 // Stored AirMini decoder short address
uint8_t  EEMEM EEAirMiniCV1Default;          // Stored AirMini decoder short address

uint8_t  EEMEM EEisSetAirMiniCV17;           // Stored AirMini decoder high byte address is set
uint8_t  EEMEM EEAirMiniCV17;                // Stored AirMini decoder high byte address
uint8_t  EEMEM EEAirMiniCV17Default;

uint8_t  EEMEM EEisSetAirMiniCV18;           // Stored AirMini decoder low byte address is set
uint8_t  EEMEM EEAirMiniCV18;                // Stored AirMini decoder low byte address
uint8_t  EEMEM EEAirMiniCV18Default;         // Stored AirMini decoder low byte address

uint8_t  EEMEM EEisSetAirMiniCV29;           // Stored AirMini decoder configuration variable is set
uint8_t  EEMEM EEAirMiniCV29;                // Stored AirMini decoder configuration variable
uint8_t  EEMEM EEAirMiniCV29Default;         // Stored AirMini decoder configuration variable

/******************************************************************************/
#ifdef DCCLibrary
///////////////////
///////////////////
///////////////////
bool NextMessage(void){  // Sets currentIndex for DCCLibrary.c's access to msg
    
    bool retval = false;

    // Set the last message extracted from the ring buffer so DCCLibrary.c can use it
    if(lastMessageExtracted != lastMessageInserted)  // Update the extract pointer because we have not caught up the the inserted pointer
    {
       lastMessageExtracted = (lastMessageExtracted+1) % msgSize;
       retval = true;
    }

    currentIndex = lastMessageExtracted;                                              // Set the variable used by DCCLibrary.c to access the msg ring buffer with no update
    memcpy((void *)&msgExtracted[0], (void *)&msg[currentIndex], sizeof(Message));    // Extract the message into private msg

    // Diagnostic output
    if (retval)
    {
       // analogWrite(LED_BUILTIN,LED_OFF);                                            // Tried, but dim
       // digitalWrite(DCC_DIAG0,0);                                                   // Will use this for diagnostics
    }
    else
    {
       // analogWrite(LED_BUILTIN,LED_ON);                                             // Tried, but dim
       // digitalWrite(DCC_DIAG0,1);                                                   // Will use this for diagnostics
    }

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

// Function, based on the value of forceDefault:
//    - TRUE:   TargetPtr's value and its related EEPROM variables are forced to use defaultValue
//    - FALSE:  extract and use EEPROM data, if previously-set, to set the TargetPtr's value
void checkSetDefaultEE(uint8_t *TargetPtr, const uint8_t *EEisSetTargetPtr, const uint8_t *EETargetPtr, uint8_t defaultValue, uint8_t forceDefault)
{
   uint8_t isSet; 
   if (EEisSetTargetPtr != (const uint8_t *)NULL) isSet = (uint8_t)eeprom_read_byte((const uint8_t *)EEisSetTargetPtr);
   else isSet = 0;
   if (!isSet || forceDefault)
   {
      *TargetPtr = defaultValue; 
      eeprom_update_byte( (uint8_t *)EEisSetTargetPtr, (const uint8_t)1 );
      eeprom_update_byte( (uint8_t *)EETargetPtr, defaultValue );
   }
   else
   {
      if(EETargetPtr != (const uint8_t *)NULL) *TargetPtr = (uint8_t)eeprom_read_byte((const uint8_t *)EETargetPtr);
      else *TargetPtr = defaultValue; 
   }
}

void setup() {

  DDRB |= 1;        // Use this for debugging if you wish
  initUART(38400);  // More debugging, send serial data out- decoded DCC packets

  ////////////////////////////////////////////////
  // Let's get the slow EEPROM stuff done first //
  ////////////////////////////////////////////////

  // Get the CHANNEL # stored in EEPROM and validate it
  eeprom_update_byte(&EECHANNELDefault, 0 );
  checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, 0, 0);      // Validate the channel, it's possible the EEPROM has bad data
  if(CHANNEL > 16) 
      checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, 0, 1);  // Force the EEPROM data to use CHANNEL 0, if the CHANNEL is invalid
  
  // Just flat out set the powerLevel
  eeprom_update_byte(&EEpowerLevelDefault, 6 );
  checkSetDefaultEE(&powerLevel, &EEisSetpowerLevel, &EEpowerLevel, 6, 1);  // Force the reset of the power level. This is a conservative approach.

  // Set the alternate DC output level to HIGH or LOW (i.e., bad CC1101 data)
  // The level of this output can be used by some decoders. The default is HIGH.
  eeprom_update_byte(&EEdcLevelDefault, 0 );
  checkSetDefaultEE(&dcLevel_in, &EEisSetdcLevel, &EEdcLevel, 1, 0);       // Use EEPROM value if it's been set, otherwise set to 1 and set EEPROM values
  dcLevel = (volatile uint8_t)dcLevel_in;                                  // Since dcLevel is volatile we need a proxy uint8_t 

  // Turn the modem OFF/ON option. For use if bad modem data is detercted in RX mode
  eeprom_update_byte(&EEturnModemOnOffDefault, 0 );
  checkSetDefaultEE(&turnModemOnOff_in, &EEisSetturnModemOnOff, &EEturnModemOnOff, 0, 0);  // Use EEPROM value if it's been set, otherwise set to 1 and set EEPROM values
  turnModemOnOff = (volatile uint8_t)turnModemOnOff_in;                                    // Needed to use a proxy variable since turnModemOnOff is volatile uint8_t

  // Set the DCC time-out period for sending IDLE packets. Used along with duplicate DCC packet detection for inserting IDLE packets
  eeprom_update_byte(&EEidlePeriodmsDefault, 0 );
  checkSetDefaultEE(&idlePeriodms, &EEisSetidlePeriodms, &EEidlePeriodms, 0, 0);  // Use EEPROM value if it's been set, otherwise set to 0 ms
  idlePeriod = (uint64_t)idlePeriodms * MILLISEC;                                 // Convert to time counts

  // Get up addressing-related CV's from EEPROM, or if not set set them in EEPROM
  eeprom_update_byte(&EEAirMiniCV1Default, 3 );
  checkSetDefaultEE(&AirMiniCV1,  &EEisSetAirMiniCV1,  &EEAirMiniCV1,    3, 0);  // Short address. By default, not using
  eeprom_update_byte(&EEAirMiniCV17Default, 227 );
  checkSetDefaultEE(&AirMiniCV17, &EEisSetAirMiniCV17, &EEAirMiniCV17, 227, 0);  // High byte to set final address to 9000
  AirMiniCV17tmp = AirMiniCV17;                                                  // Due to the special nature of CV17 paired with CV18
  eeprom_update_byte(&EEAirMiniCV18Default, 40 );
  checkSetDefaultEE(&AirMiniCV18, &EEisSetAirMiniCV18, &EEAirMiniCV18,  40, 0);  // Low byte to set final address to 9000
  eeprom_update_byte(&EEAirMiniCV29Default, 32 );
  checkSetDefaultEE(&AirMiniCV29, &EEisSetAirMiniCV29, &EEAirMiniCV29,  32, 0);  // Set CV29 so that it will use a long address
  AirMiniCV29Bit5 = AirMiniCV29 & 0b00100000;                                    // Save the bit 5 value of CV29 (0: Short address, 1: Long address)

  /////////////////////////////////
  // Initialization of variables //
  /////////////////////////////////

  startModemFlag = 0;                    // Initialize the start-modem flag
  useModemData = 1;                      // Initialize use-of-modem-data state
  DCCuseModemData(useModemData,dcLevel); // Tell the DCC code if you are or are not using modem data

  maxTransitionCount = combineHighLow(maxTransitionCountHighByte,maxTransitionCountLowByte);
  modemCVResetCount = 0;

  memset(dccptrNULL,0,sizeof(dccptrNULL));                      // Create a null dccptr for CV setting
  memset(dccptrAirMiniCVReset,0,sizeof(dccptrAirMiniCVReset));  // Initialize the reset dccptr for CV setting

  ///////////////////////////////////////////////
  // Set up the hardware and related variables //
  ///////////////////////////////////////////////

  // Set up and initialize the diagnostic LED 
  pinMode(LED_BUILTIN, OUTPUT);               // Try to set up an external LED
  // analogWrite(LED_BUILTIN,LED_OFF);          // Tried, but currently pretty dim
  digitalWrite(LED_BUILTIN,LOW);              // Tried, but currently pretty dim

  // Set up and initialize the output of the diagnostic pin (done in dccInit)
  // SET_OUTPUTPIN;                             // Set up the output diagnostic DCC pin. This is our filtered output in Rx mode
  // if(dcLevel) OUTPUT_HIGH;                   // HIGH
  // else OUTPUT_LOW;                           // LOW


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
  dccInit();                                  // Enable DCC receive
  startModem(CHANNEL, MODE);                  // Start on this Airwire Channel

  sei();                                      // enable interrupts

  then = getMsClock();                        // Grab Current Clock value for the loop below
  timeOfValidDCC = then;                      // Initialize the valid DCC data time
  inactiveStartTime = then + BACKGROUNDTIME;  // Initialize the modem idle time into the future

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

                     newIndex = (lastMessageInserted+1) % msgSize;  // Set the last message inserted into the ring buffer 

                     if((dccptrRepeatCount < dccptrRepeatCountMax) || (((int64_t)getMsClock() - lastIdleTime) < idlePeriod)) // only process if it's changed since last time
                     {                     
                       decodeDCCPacket((DCC_MSG*) dccptr);      // Send debug data
  
                       for (j=0;j<sizeof(DCC_MSG);j++)          // save for next time compare (memcpy quicker?)
                       {
                          sendbuffer[j] = dccptr[j];
#ifdef DCCLibrary
                          if (j < sizeof(DCC_MSG)-1)            // Assign msg data to send to DCCLibrary.c
                             msg[newIndex].data[j] = dccptr[j];
                          else
                             msg[newIndex].len     = dccptr[j]; // Assign msg len from the last byte to send to DCCLibrary.c
#endif
                       }
                       // digitalWrite(DCC_DIAG0,0);              // Will use this for diagnostics
                       
                     }
                     else                                       // Send out an idle packet (for keep-alive!)
                     {
                        dccptrRepeatCount = 0;
                        lastIdleTime = getMsClock();
#ifdef DCCLibrary
                        msg[newIndex].data[0] = 0xFF;
                        msg[newIndex].data[1] = 0x00;
                        msg[newIndex].data[2] = 0xFF;
                        msg[newIndex].len     = 3;
                        // digitalWrite(DCC_DIAG0,1);             // Will use this for diagnostics
#else
                        memcpy(sendbuffer,dccptr,sizeof(DCC_MSG));
#endif
                     }
#ifdef DCCLibrary
                     lastMessageInserted = newIndex;            // Update the last message inserted for comparisons. We do it here to make sure the ISR's don't affect the value
#endif

                    if(!useModemData) // If not using modem data, ensure that the output is set to a DC level after coming back from the ISR
                      {
                        if(dcLevel) OUTPUT_HIGH;                // HIGH
                        else OUTPUT_LOW;                        // LOW
                      }

                   /////////////////////////////////////////////
                   // Special processing for AirMini OPS mode //
                   /////////////////////////////////////////////
                   if((getMsClock()-opModeTime) > opModeDuration)
                     {
                           opModeTime = getMsClock();        // Reset to prevent overflow
                           // analogWrite(LED_BUILTIN,LED_OFF); // Tried, but dim
                           digitalWrite(LED_BUILTIN,LOW); // Tried, but dim
                           digitalWrite(DCC_DIAG0,0);         // Will use this for diagnostics
                           digitalWrite(DCC_DIAG1,0);         // Will use this for diagnostics
                     }
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
                            opModeTime = getMsClock();                                // We made it to ops mode for the AirMini
                            
                            if(modemCVResetCount==0 && DOUBLE_PASS)                   // Processing for identifying first or second valid call
                              {
                                 modemCVResetCount++;                                 // Update the CV reset counter
                                 memcpy(dccptrAirMiniCVReset,dccptr,sizeof(DCC_MSG)); // Save the dcc packet for comparison
                              }
                            else 
                              {
                                 if(!memcmp(dccptrAirMiniCVReset,dccptr,sizeof(DCC_MSG)) || !DOUBLE_PASS)  // If they don't compare, break out
                                   {
                                    // analogWrite(LED_BUILTIN,LED_ON);// Tried, but dim
                                    digitalWrite(LED_BUILTIN,HIGH); // Tried, but dim
                                    digitalWrite(DCC_DIAG1,1);      // Will use this for diagnostics
                                    startModemFlag = 0;             // Initialize whether the modem will be restarted
                                    tmpuint8 = dccptr[countPtr++]&(0b00000011); // zero out the first 6 bits of dccptr[countPtr], we want to use the last two bits
                                    CVnum = (uint16_t)tmpuint8;     // cast the result to a 2 byte CVnum
                                    CVnum <<= 8;                    // now move these two bit over 8 places, now that it's two bytes
                                    uint16_t tmpuint16 = (uint16_t)dccptr[countPtr++]; // cast dccptr[countPtr] to 2 bytes
                                    CVnum |= tmpuint16;             // set the last 8 bits with dccptr[countPtr]
                                    CVnum++;                        // NMRA Std of plus one, good grief, to set the final CV number
                                    CVval = dccptr[countPtr++];     // Set CVval to dccptr[countPtr], one byte only!

	                            switch(CVnum)
                                    {
                                      case  255:  // Set the channel number and reset related EEPROM values. Modest error checking. Verified this feature works
                                          if(CVval <= 16)
                                            {
                                              digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                              checkSetDefaultEE(&CHANNEL, &EEisSetCHANNEL, &EECHANNEL, CVval, 1);  // Ignore bad values
                                              startModemFlag = 1;
                                            }
                                      break;
                                      case  254:  // Set the RF power level and reset related EEPROM values. Verified this feature works.
                                          if(CVval<=10) 
                                            {
                                              digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                              checkSetDefaultEE(&powerLevel, &EEisSetpowerLevel, &EEpowerLevel, CVval, 1); // Set powerLevel and reset EEPROM values. Ignore bad values
                                              startModemFlag = 1;
                                            }
                                      break;
                                      case  253:  // Turn off/on the modem for bad packet intervals and reset related EEPROM values. Verified this feature works
                                          digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                          checkSetDefaultEE(&turnModemOnOff_in, &EEisSetturnModemOnOff, &EEturnModemOnOff, CVval, 1); // Set turnModemOnOff and reset EEPROM values
                                          turnModemOnOff = (volatile uint8_t)turnModemOnOff_in; // Assign for volatile
                                      break;
                                      case  252:  // Set the tooLong (in quarter second intervals) and reset related EEPROM values. 
                                          digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                          tooLong = CVval*QUARTERSEC;
                                      break;
                                      case  251:  // Set the sleepTime (in quarter second intervals) and reset related EEPROM values. Verified this feature works.
                                          digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                          sleepTime = CVval*QUARTERSEC;
                                      break;
                                      case  250:  // Set the low byte for transition counts
                                          digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                          maxTransitionCountLowByte = CVval;
                                          maxTransitionCount = combineHighLow(maxTransitionCountHighByte,maxTransitionCountLowByte);
                                      break;
                                      case  249:  // Set the high byte for transition counts
                                          digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                          maxTransitionCountHighByte = CVval;
                                          maxTransitionCount = combineHighLow(maxTransitionCountHighByte,maxTransitionCountLowByte);
                                      break;
                                      case  248:  // Set the DC output level and reset related EEPROM values. Verified this feature works.
                                          digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                          checkSetDefaultEE(&dcLevel_in, &EEisSetdcLevel, &EEdcLevel, CVval, 1); // Set dcLevel and reset EEPROM values
                                          dcLevel = (volatile uint8_t)dcLevel_in;
                                      break;
                                      case  247:  // Set the idle period (in ms) and reset related EEPROM values. Verified it works.
                                          digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                          checkSetDefaultEE(&idlePeriodms, &EEisSetidlePeriodms, &EEidlePeriodms, CVval, 1); // Set idlePeriodms and reset EEPROM values (in ms!)
                                          idlePeriod = idlePeriodms * MILLISEC; // Convert to cycles
                                      break;
                                      case 29:    // Set the Configuration CV and reset related EEPROM values. Verified this feature works.
                                          digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                          checkSetDefaultEE(&AirMiniCV29, &EEisSetAirMiniCV29, &EEAirMiniCV29, CVval, 1); // Set AirMiniCV29 and reset EEPROM values
                                          AirMiniCV29Bit5 = AirMiniCV29 & 0b00100000; // Save the bit 5 value of CV29 (0: Short address, 1: Long address)
                                      break;
                                      case 18:    // Set the Long Address Low Byte CV and reset related EEPROM values. Verified this feature works.
                                          digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                          checkSetDefaultEE(&AirMiniCV17, &EEisSetAirMiniCV17, &EEAirMiniCV17, AirMiniCV17tmp, 1); // Changes not take effect until now
                                          checkSetDefaultEE(&AirMiniCV18, &EEisSetAirMiniCV18, &EEAirMiniCV18, CVval, 1); 
                                      break;
                                      case 17:    // Set the Long Address High Byte CV and save values after validation (do NOT write to AirMini's CV17 or EEPROM yet!). Verified this feature works
                                          if((0b11000000<=CVval) && (CVval<=0b11100111))  // NMRA standard 9.2.2, Paragraphs 129-135
                                            {
                                              digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                              AirMiniCV17tmp = CVval;    // Do not take effect until CV18 is written! NMRA Standard 9.2.1, footnote 8.
                                            }
                                      break;
                                      case 1:     // Set the Short Address CV and reset related EEPROM values after validation. Verified this feature works.
                                          if((0<CVval) && (CVval<128))  // CV1 cannot be outside this range. Some decoders limit 0<CVval<100
                                            {
                                              digitalWrite(DCC_DIAG0,1); // Will use this for diagnostics
                                              checkSetDefaultEE(&AirMiniCV1, &EEisSetAirMiniCV1, &EEAirMiniCV1, CVval, 1); // Set AirMiniCV1 and reset EEPROM values. Ignore bad values
                                            }
                                      break;
                                    } // end of switch(CVnum) 

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

                         else modemCVResetCount=0; // Reset this counter if we didn't get a "hit" on CV changing

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

        if((MODE==RX) && !useModemData)        // If not using modem data, ensure the output is set to DC after coming back from the ISR
          {
            if(dcLevel) OUTPUT_HIGH;           // HIGH
            else OUTPUT_LOW;                   // LOW
          }
        
         if( now > BACKGROUNDTIME )            // Check for Time Scheduled Tasks
           {                                   // A priority Schedule could be implemented in here if needed
              then = getMsClock();             // Grab Clock Value for next time

              if((MODE==TX) || useModemData)
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
                   if((MODE==RX) && ((then-timeOfValidDCC) >= tooLong) && (getTransitionCount() > maxTransitionCount))
                     {
                       if (turnModemOnOff) sendReceive(STOP); // send stop command to modem if no DEMUX is available
                       useModemData = 0;                      // false use-of-modem-data state
                       DCCuseModemData(useModemData,dcLevel); // Tell the DCC code if you are or are not using modem data
                       // analogWrite(LED_BUILTIN,LED_OFF);      // Tried, but dim
                       inactiveStartTime = then;              // Start inactive timer
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
              else if((MODE==RX) && 
                      (((then-inactiveStartTime) >= sleepTime) || ((then-timeOfValidDCC) < tooLong)))
                {
                   useModemData = 1;                          // Active use-of-modem-data state
                   DCCuseModemData(useModemData,dcLevel);     // Tell the DCC code if you are or are not using modem data
                   // analogWrite(LED_BUILTIN,LED_ON);        // Tried, but dim
                   timeOfValidDCC = then;                     // Start over on the DCC timing
                   inactiveStartTime = then + BACKGROUNDTIME; // start the modem inactive timer sometime in the future
                   if(turnModemOnOff)
                     {
                       dccInit();                             // Reset the DCC state machine, which also resets transitionCount
                       sendReceive(MODE);                     // awaken the modem in MODE
                     }
                   else resetTransitionCount(0);              // While we haven't reset the DCC state machine, do restart transitionCount
                }

              PORTB ^= 1;                      // debug - monitor with logic analyzer
          }
}


