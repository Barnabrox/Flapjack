// PIN LAYOUT:
// RX0 - Roboclaw S2 (Serial1)
// TX1 - Roboclaw S1 (Serial1)
// A7  - Wedge Potentiometer
// D7  - RC Receiver Data Line (Serial2)
// D9  - Tooth Motor Controller IN1
// D10 - Tooth Motor Controller IN2

// CONTROLLER CHANNELS
// Ch1, chData[0] - Right Stick X axis
// Ch2, chData[1] - Right Stick Y axis
// Ch3, chData[2] - Left Stick Y axis
// Ch4, chData[3] - Left Stick X axis
// Ch5, chData[4] - VRA
// Ch6, chData[5] - VRB
// Ch7, chData[6] - SWA
// Ch8, chData[7] - SWB
// Ch9, chData[8] - SWC
// Ch10 chData[9] - SWD

// WEDGE RANGE
//  190 - Bottom
//  700 - Top
// 1023 - 3v Battery Disconnected

// Include Libraries
#include <ibus.h>
#include <RoboClaw.h>

// Declare Roboclaw variables
RoboClaw rc(&Serial1, 10000); // Create a new RoboClaw object (rc) to handle communication to/from the motor controllers
const char rcDrive = 0x80;    // Address of the motor controller connected to the drive motors
const char rcWedge = 0x81;    // Address of the motor controller connected to the wedge motors

// Declare ibus variables
struct ibus_state ibus_data;  // ibus data structure (for RC receiver communication)
const int numCh = 10;         // Number of chanels to read
uint16_t chData[numCh];       // Holds data from the RC receiver for each of the 10 chanels

// Specify Wedge Potentiometer Pin
int wedgePotPin = A7;

// Specify Tooth Motor Controller Pins
int toothPin1 = 9;
int toothPin2 = 10;

// Declare variables to hold the state of the Tooth Switch (ts) on the controller
int tsOldState = -1;
int tsNewState = -1;
long tsStateChangeTime = 0;

// Declare variables to hold the state of inversion
int invOldState = 0;
int invNewState = 0;

void setup() {
  
  // Start serial interfaces
  rc.begin(230400);       // Start serial interface to the speed controllers
  Serial2.begin(115200);  // Start serial interface to RC receiver

  // Initialize the ibus data structure
  ibus_init(&ibus_data, numCh);

  // Set pin modes
  pinMode(wedgePotPin, INPUT);
  pinMode(toothPin1, OUTPUT);
  pinMode(toothPin2, OUTPUT);
  digitalWrite(toothPin1, HIGH);
  digitalWrite(toothPin2, HIGH);

  // Set analog reference to 3.3v coin cell battery
  analogReference(EXTERNAL);
  
  // Uncomment for debugging
  Serial.begin(115200);   // Start serial interface to PC (debug)
  // Serial3.begin(115200);  // Start serial interface to Bluetooth
}

void loop() {
  
  // If new data is available from the RC receiver...
  if (Serial2.available() > 0) {
    // Process it, and if it is the last byte of the current message...
    if (ibus_read(&ibus_data, chData, Serial2.read()) == 0) {

      // INVERTED CONTROLS //
      //===================//

      if (chData[6] > 1500) {

        invOldState = invNewState;
        invNewState = 1;

        // Drive
        rc.DutyM1M2(rcDrive, -ConvertToDutyCycle(chData[2]), -ConvertToDutyCycle(chData[1]));

        // Wedge
        rc.DutyM1M2(rcWedge, -ConvertToDutyCycle(chData[5]), -ConvertToDutyCycle(chData[5]));

        // Teeth
        if (invOldState == 0 and invNewState == 1){
          digitalWrite(toothPin1, LOW);
          analogWrite( toothPin2, 255); 
        }
        
      }

      // STANDARD CONTROLS //
      //===================//
      
      else {

        invOldState = invNewState;
        invNewState = 0;

        // Drive Control //
        //===============//
  
        rc.DutyM1M2(rcDrive, ConvertToDutyCycle(chData[1]), ConvertToDutyCycle(chData[2]));

        // Wedge Control //
        //===============//
        
        rc.DutyM1M2(rcWedge, ConvertToDutyCycle(chData[5]), ConvertToDutyCycle(chData[5]));
        Serial.println(analogRead(wedgePotPin));
  
        // Tooth control //
        //===============//
  
        // If it's the first time through the loop...
        if (tsOldState == -1) {
          // Set the old and new states to the same value
          tsOldState = chData[9];
          tsNewState = chData[9];
        }
        // Otherwise...
        else {
          // Store the old switch state and load the new switch state
          tsOldState = tsNewState;
          tsNewState = chData[9];
        }
  
        // If the switch has been fliped from low to high...
        if (tsNewState < 1500 and tsOldState >= 1500) {
          // Raise the teeth
          digitalWrite(toothPin1, HIGH);
          digitalWrite(toothPin2, LOW);
          tsStateChangeTime = millis();
        }
        // If the switch has been fliped from high to low...
        else if (tsNewState > 1500 and tsOldState <= 1500) {
          // Lower the teeth
          digitalWrite(toothPin1, LOW);
          digitalWrite(toothPin2, HIGH);
          tsStateChangeTime = millis();
        }
        // If more than 250ms has elapsed since the last switch change...
        else if (millis() - tsStateChangeTime > 250) {
          // Stop the tooth motor
          digitalWrite(toothPin1, HIGH);
          digitalWrite(toothPin2, HIGH);
        }
      }
    }
  }
}

int ConvertToDutyCycle(uint16_t chData) {
  if (chData < 1000 or chData > 2000) return 64;  // If chData is not in expected range, stop motors
  int x = ((int)chData - 1500) * 65.534;          // Convert from 1000-2000 to (-32767)-(32767)
  return x;
}
