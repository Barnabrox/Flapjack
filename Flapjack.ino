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

// WEDGE DUTY CYCLE (for the rc.DutyM1M2 function)
// "Positive" duty cycle moves the wedge up
// "Negative" duty cycle moves the wedge up

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

// Declare PID variables
int sp = 450, pv = 450;
unsigned long dt = 0;
double pid_err, pid_int, pid_der, pid_out, Kp2;
const double Kp = -150;
const double Ki = -0.5;
const double Kd = -3000;
double wedgeMotorOut;

// Derivative Averaging
bool done_charging = false;
int i=0, j=-1;
const int hist_size = 5;
double pv_hist[hist_size][2];
double sumT, sumPV, aveT, avePV, devT, devPV, sumRise, sumRun;

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

        // Adjust Set Point
        if (chData[5] > 1750) {sp = 550;}
        else if (chData[5] < 1250) {sp = 360;}
        else {sp = 420;}

        // Increment Array Counter
        j++; 
        if (j >= hist_size){
          j=0;
          done_charging = true;
        }
        
        // Read Current Time and Process Variable
        pv_hist[j][0] = millis();
        pv_hist[j][1] = analogRead(wedgePotPin);

        // If the 3v battery is disconnected, disable PID wedge control
        // Don't re-enable until the battery has been connected for 10 loop iterations
        if (pv_hist[j][1] == 1023){
          done_charging = false;
          j = 0;
          pv_hist[j][0] = millis();
          pv_hist[j][1] = analogRead(wedgePotPin);
        }

        // Calculate Time Elapsed
        if (j==0) {dt = pv_hist[j][0] - pv_hist[hist_size-1][0];}
        else {dt = pv_hist[j][0] - pv_hist[j-1][0];}

        // Scale down the porpotional constant when the wedge is moving down
        if (pid_err > 0) {Kp2 = 0.5;}
        else {Kp2 = 1;}

        // Calculate average t and pv
        sumT = 0; sumPV = 0;
        for (i=0; i<hist_size; i++){
          sumT  += pv_hist[i][0];
          sumPV += pv_hist[i][1];
        }
        aveT  = sumT  / hist_size;
        avePV = sumPV / hist_size;

        // Calculate best-fit derivative term
        sumRise = 0; sumRun = 0;
        for (i=0; i<hist_size; i++){
          devT  = pv_hist[i][0] - aveT;
          devPV = pv_hist[i][1] - avePV;
          sumRise += devT * devPV;
          sumRun  += devT * devT;
        }
        pid_der = sumRise/sumRun;

        // Calculate the P & I terms
        pid_err = pv_hist[j][1] - sp;
        if (abs(pid_err) < 8) {pid_err = 0;}
        pid_int = pid_int + pid_err * dt;

        // Calculate the output of the PID loop by summing and scaling the P,I,D terms
        pid_out = Kp*Kp2*pid_err + Ki*pid_int + Kd*pid_der;

        wedgeMotorOut = pid_out;
        wedgeMotorOut = min(32767, wedgeMotorOut);
        wedgeMotorOut = max(-32767, wedgeMotorOut);
        if (pv > 600) {wedgeMotorOut = min(0, wedgeMotorOut);}
        if (pv < 300) {wedgeMotorOut = max(0, wedgeMotorOut);}

        // Only output to the motors if the pv_hist array is full
        if (done_charging){
          rc.DutyM1M2(rcWedge, (int)wedgeMotorOut, (int)wedgeMotorOut);
        }
        else{
          rc.DutyM1M2(rcWedge, 0, 0);
        }

        Serial.print(pv_hist[j][1]); Serial.print("\t");
        Serial.print(sp); Serial.print("\t");
        // Serial.print(Kp*pid_err); Serial.print("\t");
        // Serial.print(Ki*pid_int); Serial.print("\t");
        // Serial.print(Kd*pid_der); Serial.print("\t");
        // Serial.print(pid_out); Serial.print("\t");
        // Serial.print(dt); Serial.print("\t");
        // Serial.print(wedgeMotorOut); Serial.print("\t");
        Serial.println();
  
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
