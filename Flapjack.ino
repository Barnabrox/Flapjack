// PIN LAYOUT:
// TX1 - Roboclaw S1 (Serial1)
// RX0 - Roboclaw S2 (Serial1)
// D7  - RC Receiver Data Line (Serial2)
// D9  - Tooth Motor Controller IN1
// D10 - Tooth Motor Controller IN2

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

// Specify Tooth Motor Controller Pins
int toothPin1 = 9;
int toothPin2 = 10;

// Declare variables to hold the state of the Tooth Switch (ts) on the controller
int tsOldState = -1;
int tsNewState = -1;
long tsStateChangeTime = 0;

void setup() {
  
  // Start serial interfaces
  rc.begin(230400);       // Start serial interface to the speed controllers
  Serial.begin(115200);   // Start serial interface to PC (debug)
  Serial2.begin(115200);  // Start serial interface to RC receiver
  Serial3.begin(115200);  // Start serial interface to Bluetooth

  // Initialize the ibus data structure
  ibus_init(&ibus_data, numCh);

  // Set pin modes
  pinMode(toothPin1, OUTPUT);
  pinMode(toothPin2, OUTPUT);
  digitalWrite(toothPin1, HIGH);
  digitalWrite(toothPin2, HIGH);
}

void loop() {
  
  // If new data is available from the RC receiver...
  if (Serial2.available() > 0) {
    // Process it, and if it is the last byte of the current message...
    if (ibus_read(&ibus_data, chData, Serial2.read()) == 0) {

      // Drive and Wedge Control //
      //=========================//
      
      rc.DutyM1M2(rcDrive, ConvertToDutyCycle(chData[1]), ConvertToDutyCycle(chData[2]));
      rc.DutyM1M2(rcWedge, ConvertToDutyCycle(chData[5]), ConvertToDutyCycle(chData[5]));

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
      // If more than 1s has elapsed since the last switch change...
      else if (millis() - tsStateChangeTime > 250) {
        // Stop the tooth motor
        digitalWrite(toothPin1, HIGH);
        digitalWrite(toothPin2, HIGH);
      }
      
      // Diagnostics //
      //=============//
      
      //ReadDiagnostics(rcDrive);
    }
  }
}

int ConvertToDutyCycle(uint16_t chData) {
  if (chData < 1000 or chData > 2000) return 64;  // If chData is not in expected range, stop motors
  int x = ((int)chData - 1500) * 65.534;          // Convert from 1000-2000 to (-32767)-32767
  return x;
}

void ReadDiagnostics(char adr) {

  // Decare variables
  int batt = -1;                    // Main battery voltage
  int pwm1 = -1, pwm2 = -1;         // PWM outputs for M1,M2
  int cur1 = -1, cur2 = -1;         // Current for M1,M2
  uint16_t temp1 = -1, temp2 = -1;  // Board temp
  long stat = -1;                   // Status (see manual)
  bool valid = true;                // Valid data flag

  // Poll the roboclaw for status
  // 4.6ms at 115200 baud
  // 2.6ms at 230400 baud
  // 1.9ms at 460800 baud
  batt = rc.ReadMainBatteryVoltage(adr, &valid);
  if (valid == false) batt = -1;
  rc.ReadPWMs(adr, pwm1, pwm2);
  rc.ReadCurrents(adr, cur1, cur2);
  if (rc.ReadTemp(adr, temp1) == false) temp1 = -1;
  if (rc.ReadTemp2(adr, temp2) == false) temp2 = -1;
  stat = rc.ReadError(adr, &valid);
  if (valid == false) stat = -1;

  // Print to console
  // This takes ~1.1ms at 115200 baud
  Serial3.print(millis());
  Serial3.print("\t");
  Serial3.print(batt);
  Serial3.print("\t");
//  Serial3.print(pwm1);
//  Serial3.print("\t");
//  Serial3.print(pwm2);
//  Serial3.print("\t");
  Serial3.print(cur1);
  Serial3.print("\t");
  Serial3.print(cur2);
  Serial3.print("\t");
  Serial3.print(temp1);
  Serial3.print("\t");
  Serial3.print(temp2);
  Serial3.print("\t");
  Serial3.println(stat, HEX);
}
