/*
    Name:       Turnout_Routher.ino
    Created:	Sun Feb  8 10:57:18 EST 2026
    Author:     Darrell Lamm

	DEPENDENCIES:
  TBD

	DESCRIPTION:	 
  Automatic route selection

	HARDWARE notes:

	COMMANDS:
	set-up command is
                         <------------- Turnout # ----------------->
                         0     1     2     3     4     5    6    7
	s route_number(0-7), [ctx], [ctx], [ctx], [ctx], [ctx], [ctx],[ctx],[ctx] 
  

	execute command is
	r route_number(0-7)

	possible FUTURES:
	Add direct pushbutton control.
	build lighting effects, say on/off control for night lighting/ fires/ welding

	Why not use arcomora software?  It won't fit into a nano with a standard bootloader.
	I also considered it too complex to set up.  Also its not clear if it can support the simple 
	solution used here where the nano is plugged direct into a low cost expansion board.


	https://github.com/nzin/arduinodcc/tree/master/arduinoSource/dccduino
	https://rudysmodelrailway.wordpress.com/2015/01/25/new-arduino-dcc-servo-and-function-decoder-software/
	https://www.arcomora.com/mardec/
	
	   
*/
#include <EEPROM.h>

#define MAX_TURNOUTS 8
#define MAX_ROUTES 32

#define A0_PIN 2
#define A1_PIN 3
#define A2_PIN 4
#define DATA_PIN 5
#define ENABLE_PIN 6
#define CL_PIN 7
#define B0_PIN 8
#define B1_PIN 9
#define B2_PIN 10
#define EO_PIN 11

/*EEprom and version control*/
struct CONTROLLER
{
	long	softwareVersion = 20260208;  //yyyymmdd captured as an integer
	bool isDirty=false;  //will be true if eeprom needs a write
	long long padding;
};

/*defaultController contains defaults defined in the CONTROLLER object and is used to override eeprom settings
on new compilations where data structures have changed.  The system runs off the m_bootController object normally
the padding variable seems to fix a bug on the nano that saw eeprom contents corrupt on readback*/
CONTROLLER bootController;
CONTROLLER m_defaultController;

//declare EEPROM functions
void getSettings(void);
void putSettings(void);


//serial communication related
void recvWithEndMarker();
const byte numChars = 64;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;

bool ledState;

typedef enum {
	TURNOUT_THROWN=1,
	TURNOUT_CLOSED=2,
	TURNOUT_DONTCARE=0,
} switchState;

typedef uint8_t VIRTUALROUTE[MAX_TURNOUTS];

//Up to eight routes
VIRTUALROUTE virtualroute[MAX_ROUTES];
VIRTUALROUTE *vrBoot=nullptr;


unsigned long currentMs;
unsigned long previousMs;
uint8_t bootTimer=0;
uint8_t tick;
uint8_t input_address = 7;
uint8_t input_address_old;
uint8_t b0, b1, b2, eo;
uint8_t turnout_state;
uint8_t global_turnout_state[MAX_TURNOUTS] = {0,0,0,0,0,0,0,0};

void helpPrint() {
     Serial.println("============================================================================================");
     Serial.println("Commands");
     Serial.println("Help (this message):         h or ?");
     Serial.println("Set route:                   s route_number(0-7) turnout_state0(0-2) ... turnout_state7(0-2)");
     Serial.println("Examine route or all routes: x route_number(0-7) or x");
     Serial.println("Examine all turnout states:  e ");
     Serial.println("Activate route:              r route_number(0-7)");
     Serial.println("Directly set turnout:        t turnout_number(0-7) turnout_state(0-2)");
     Serial.println("turnout_state values: ");
     Serial.println("   0: do nothing");
     Serial.println("   1: set turnout direction 1");
     Serial.println("   2: set turnout direction 2");
     Serial.println("============================================================================================");
}

void setup() {
	Serial.begin(115200);
	delay(1000);
	
	//restore virtualroute array from EEPROM
	getSettings();
	Serial.println("boot\n");
  helpPrint();
	pinMode(LED_BUILTIN, OUTPUT);

  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(CL_PIN, OUTPUT);
  pinMode(B0_PIN, INPUT_PULLUP);
  pinMode(B1_PIN, INPUT_PULLUP);
  pinMode(B2_PIN, INPUT_PULLUP);
  pinMode(EO_PIN, INPUT_PULLUP);

  digitalWrite(ENABLE_PIN, HIGH); // Disable latching initially
  digitalWrite(CL_PIN, LOW); // Disable CL initially
  b0 = digitalRead(B0_PIN);
  b1 = digitalRead(B1_PIN);
  b2 = digitalRead(B2_PIN);
  eo = digitalRead(EO_PIN);

  if (eo) input_address = 7-((b2<<2) | (b1<<1) | (b0));
  Serial.print("Initial input address: ");
  Serial.println(input_address,DEC);

}

void latchWrite(uint8_t turnout_number, uint8_t turnout_state) {
  // Set the address pins
  if (!turnout_state) return;
  global_turnout_state[turnout_number] = turnout_state;
  Serial.print("turnout_number: ");
  Serial.print(turnout_number,DEC);
  Serial.print(" set to turnout_state: ");
  Serial.println(turnout_state);
  digitalWrite(A0_PIN, (turnout_number & B001));
  digitalWrite(A1_PIN, (turnout_number & B010) >> 1);
  digitalWrite(A2_PIN, (turnout_number & B100) >> 2);

  // Set the data pin
  digitalWrite(DATA_PIN, turnout_state-1); // Note the "-1"

  // Pulse the enable pin to latch the turnout_state
  digitalWrite(ENABLE_PIN, LOW);
  delay(1);
  digitalWrite(ENABLE_PIN, HIGH);
}

void routeWrites(uint8_t route_number) {
  Serial.print("Activating route ");
  Serial.println(route_number);
  for (auto turnout_number = 0; turnout_number < MAX_TURNOUTS ; turnout_number++) {
     turnout_state = (virtualroute[route_number])[turnout_number];
     latchWrite(turnout_number,turnout_state);
  }
}

void loop() {

  // Pole the input address pins
  delay(100);
  input_address_old = input_address;
  b0 = digitalRead(B0_PIN);
  b1 = digitalRead(B1_PIN);
  b2 = digitalRead(B2_PIN);
  eo = digitalRead(EO_PIN);
  if (eo) input_address = 7-((b2<<2) | (b1<<1) | (b0));
  // input_address = 7 - 4*b2 - 2*b1 - b0;
  /*
  Serial.print("b2:b1:b0=");
  Serial.print(b2,DEC);Serial.print(":");
  Serial.print(b1,DEC);Serial.print(":");
  Serial.print(b0,DEC); Serial.print(" -> ");
  Serial.println(input_address,DEC);
  */
  if (input_address_old != input_address) {
     Serial.print("Changed input address: ");
     Serial.println(input_address);
     routeWrites(input_address);
  }
  
	//look for more incoming serial data
	recvWithEndMarker();

	//process serial command
	if (newData) {
		//Serial.println(receivedChars);
		newData = false;

		//look for command s route_number, turnout_state0, ..., turnout_state7 (max)
		if (receivedChars[0] == 's') {
           VIRTUALROUTE vrParse = {0, 0, 0, 0, 0, 0, 0, 0};
           //detokenize
           char *pch;
           uint8_t i = 0;
           uint8_t route_number = MAX_ROUTES;
           uint8_t turnout_state = 3;
           pch = strtok(receivedChars, " ,");
           pch = strtok(NULL, " ,");
           while (pch != NULL) {
              if (i==0) {
                 route_number = atoi(pch);
                 //Valid route numbr
                 if (route_number >= MAX_ROUTES) {
                    i = 10;  //error code
                    Serial.println("bad route number (0-7)");
				         }
               }
               else if (i <= MAX_TURNOUTS) {
                 turnout_state = atoi(pch);
                 if (turnout_state>2) {
                    i = 10;  //error code
                    Serial.println("bad turnout state (0-2)");
                 }
                 else {
                    vrParse[i-1] = turnout_state;
                 }
              }
              //zero is the s char

		          ++i;
		          pch = strtok(NULL, " ,");

            } // end of while

			if ((i<=1) || (i >=10)) {
               Serial.println("bad command. usage s route_number, turnout_state0, .., turnout_state7");
			}
			else {
               Serial.println("OK");
               //match to a route number copy it over
               Serial.print("Loading route ");
               Serial.println(route_number, DEC);
               for (auto turnout_number = 0; turnout_number < MAX_TURNOUTS ; turnout_number++) {
                  (virtualroute[route_number])[turnout_number] = vrParse[turnout_number];
                  Serial.print(" turnout ");
                  Serial.print(turnout_number,DEC);
                  Serial.print(": ");
                  Serial.println((virtualroute[route_number])[turnout_number],DEC);
               }

               bootController.isDirty = true;
               putSettings();

		    }
		
     } // 's'

	//look for command r route_number  
	if (receivedChars[0] == 'r') {
       //detokenize
       char *pch;
       pch = strtok(receivedChars, " ,");
       pch = strtok(NULL, " ,");
       if (pch != NULL) {
          uint8_t route_number = atoi(pch);
          if (route_number < MAX_ROUTES) {
             Serial.println("OK");
             routeWrites(route_number);
          }
          else {
             Serial.println("bad command. usage r route_number(0-7)");
          }
       }
       else {
          Serial.println("bad command. usage r route_number(0-7)");
       }

    } // 'r'
	
	//look for direct turnout command
	if (receivedChars[0] == 'e') {
    for (uint8_t turnout_number = 0; turnout_number < MAX_TURNOUTS; turnout_number++){
       Serial.print("turnout ");
       Serial.print(turnout_number,DEC);
       Serial.print(": ");
       Serial.println(global_turnout_state[turnout_number],DEC);
    }
  } // 'e'

	//look for direct turnout command
	if (receivedChars[0] == 't') {
       //detokenize
       char *pch;
       pch = strtok(receivedChars, " ,");
       pch = strtok(NULL, " ,");
       uint8_t error = 0;
       if (pch != NULL) {
          uint8_t turnout_number = atoi(pch);
          if (turnout_number < MAX_TURNOUTS) {
             Serial.println("OK");
             pch = strtok(NULL, " ,");
             uint8_t turnout_state = atoi(pch);
             if (turnout_state <=2) {
                latchWrite(turnout_number,turnout_state);
             }
             else
                error = 1;
          }
          else {
             error = 1;
          }
       }
       else {
          error = 1;
       }
       
       if (error) Serial.println("bad command. usage t turnout_number(0-7) turnout_state(0-2)");

    } // 't'

  	//look for help
  	if (receivedChars[0] == '?' || receivedChars[0] == 'h') {
       helpPrint();
    } // '?' or 'h'
	
		//2020-08-31 dump all servos
		//keep it consistent with s command; pin,addr,swing,invert,continuous

    if (receivedChars[0] == 'x') {
      uint8_t i = 0;
      uint8_t imatch = MAX_ROUTES;
      char *pch;
      pch = strtok(receivedChars, " ,");
      pch = strtok(NULL, " ,");
      if (pch!=NULL) imatch = atoi(pch);
      if (imatch >= MAX_ROUTES) imatch = MAX_ROUTES;
      for (auto vr : virtualroute) {
        if (i==imatch || imatch==MAX_ROUTES) {
           Serial.print("Examining route ");
           Serial.println(i, DEC);
           for (auto turnout_number = 0; turnout_number < MAX_TURNOUTS; turnout_number++){
              Serial.print(" turnout ");
              Serial.print(turnout_number,DEC);
              Serial.print(": ");
              Serial.println(vr[turnout_number], DEC);
           }
        }
        i++;
      }
    } // 'x'

	}//newData

}//main loop

//https://forum.arduino.cc/index.php?topic=288234.0
void recvWithEndMarker() {
	static byte ndx = 0;
	char endMarker = '\n';
	char rc;

	// if (Serial.available() > 0) {
	while ((Serial.available() > 0) && (newData == false)) {
		rc = Serial.read();
		
		if (rc != endMarker) {
			receivedChars[ndx] = rc;
			ndx++;
			if (ndx >= numChars) {
				ndx = numChars - 1;
			}
		}
		else {
			receivedChars[ndx] = '\0'; // terminate the string
			ndx = 0;
			newData = true;
		}
	}
}


//getSettings reads from EEPROM. nano has 1024 bytes of eeprom
//https://raw.githubusercontent.com/RuiSantosdotme/Random-Nerd-Tutorials/master/Projects/Arduino_EEPROM.ino

void getSettings(void) {
	int eeAddr = 0;
	EEPROM.get(eeAddr, bootController);
	if (m_defaultController.softwareVersion != bootController.softwareVersion) {
		/*If software version has changed, we need to re-initiatise eeprom with factory defaults*/
		Serial.println("restore factory defaults");
		EEPROM.put(0, m_defaultController);
		eeAddr += sizeof(m_defaultController);
		
		/*use pin 3 onward. set defaults*/
		for (auto & r : virtualroute) {
       for(auto turnout_number=0; turnout_number < MAX_TURNOUTS; turnout_number++) {
          r[turnout_number] = 0;
       }
		}
		/*write back default values*/
		EEPROM.put(eeAddr, virtualroute);
	}

	/*either way, now populate our structs with EEprom values*/
	eeAddr = 0;
	EEPROM.get(eeAddr, bootController);
	eeAddr += sizeof(bootController);
	EEPROM.get(eeAddr, virtualroute);
	
	Serial.print("\nSofware version: ");
	Serial.print(bootController.softwareVersion, DEC);
	Serial.println("\n.............\n");

}


//putSettings writes to EEPROM. to reduce wear only call if a change has been made
void putSettings(void) {
	int eeAddr = 0;
	if (bootController.isDirty == false) { return; }
	EEPROM.put(eeAddr, bootController);
	eeAddr += sizeof(bootController);
	EEPROM.put(eeAddr, virtualroute);
	Serial.println("putSettings");
	bootController.isDirty = false;
}
