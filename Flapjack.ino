//=============//
// DESCRIPTION //
//=============//

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
// Ch5, chData[4] - KEY1 (wedge bumper - up)
// Ch6, chData[5] - KEY2 (wedge bumper - down)
// Ch7, chData[6] - SWA  (inversion)
// Ch8, chData[7] - SWB  (wedge operational mode {manual/automatic})
// Ch9, chData[8] - SWC
// Ch10 chData[9] - SWD (teeth)

// WEDGE RANGE
// 1020 - Sensor Disconnected
//  710 - Top of Range
//  582 - Resting on ground (inverted)
//  540 - Center
//  500 - Resting on ground (standard orientation)
//  400 - Bottom of Range

// WEDGE DUTY CYCLE (for the rc.DutyM1M2 function)
// "Positive" duty cycle moves the wedge up
// "Negative" duty cycle moves the wedge up

//=======//
// TO DO //
//=======//

// Nothing

//===========//
// LIBRARIES //
//===========//

#include <ibus.h>
#include <RoboClaw.h>

//===========//
// VARIABLES //
//===========//

// Roboclaw variables
RoboClaw rc(&Serial1, 10000); // Create a new RoboClaw object (rc) to handle communication to/from the motor controllers
const char rcDrive = 0x80;    // Address of the motor controller connected to the drive motors
const char rcWedge = 0x81;    // Address of the motor controller connected to the wedge motors

// ibus variables
struct ibus_state ibus_data;  // ibus data structure (for RC receiver communication)
const int numCh = 10;         // Number of chanels to read
uint16_t chData[numCh];       // Holds data from the RC receiver for each of the 10 chanels

// GPIO pin assignments
const int PIN_ENC_CS = 8 ;    // Digital encoder chip select pin
const int PIN_ENC_DATA = 11;  // Digital encoder data pin
const int PIN_ENC_CLOCK = 12; // Digital encoder clock pin
const int PIN_TOOTH1 = 9;     // Tooth motor controller input 1 pin
const int PIN_TOOTH2 = 10;    // Tooth motor controller input 2 pin

// Motor output variables
int pwm_DriveMotorR = 0;
int pwm_DriveMotorL = 0;
double pwm_WedgeMotors = 0;

// Robot orientation variables
enum InversionStates {STANDARD, INVERTED};
InversionStates state = STANDARD;
InversionStates prev_state = STANDARD;

// Wedge control mode variables
enum WedgeModes {MANUAL, AUTOMATIC};
WedgeModes wedgeMode = MANUAL;
WedgeModes prev_wedgeMode = MANUAL;
bool safety_override = false;

// Wedge manual control speed scaling variables
double manualWedgeModeSpeedScalingUp   = 0.5;
double manualWedgeModeSpeedScalingDown = 0.3;
double manualWedgeModeSpeedScaling;

// Tooth motor command variables
enum ToothMotorCommands {RAISE, LOWER, STOP};
ToothMotorCommands toothMotorCommand = STOP;

// Tooth switch position variables
enum ToothSwitchPositions {UP, DOWN, UNDEFINED};
ToothSwitchPositions prevToothSwitchPosition = UNDEFINED;
ToothSwitchPositions currToothSwitchPosition = UNDEFINED;

// PID Variables
const int ave_count = 5;    // Number of previous points used to calculate the best fit slope for the derivative term [d(pv)/dt]
int sp = 540;               // Set Point
int sp_prev = 540;          // previous set point
int pv[ave_count];          // Process Variable
unsigned long t[ave_count]; // Time (millis)
unsigned long dt = 0;       // Time interval; used when calculating the derivative and integral terms
double pid_err, pid_int, pid_der; // Proportional, Integral, and Derivative terms
double sumT, sumPV, aveT, avePV, devT, devPV, sumRise, sumRun;  // Used for calculating the best fit slope
bool done_charging = false; // Becomes true when enough valid points are captured for averaging
int i=0, j=-1;              // Loop counters
double Kp2;                 // Used to scale Kp based on if the wedge is moving up or down

// Timing Variables
unsigned long watchdogTimeout = 0;        // The last time new data arrived from the reciever
unsigned long tsStateChangeTime = 0;      // The last time the tooth switch changed state
unsigned long pid_int_enable_time = 0;    // The time after which to enable the integral term in the PID loop
unsigned long lastLoopTime = 4294967295;  // The last time the 10ms loop was ran
unsigned long move_wedge_t0 = 0;          // The time the wedge was first commanded to move in manual mode
bool bNewReceiverData = false;            // Becomes true when new data arrives from the reciever

// PID tuning constants
const double Kp = -240;   // -150
const double Ki = -0.8;   // -0.8
const double Kd = -8000;  // -3000

//=======//
// SETUP //
//=======//

void setup() {
  
  // Start serial interfaces
  rc.begin(230400);       // Start serial interface to the speed controllers
  Serial2.begin(115200);  // Start serial interface to RC receiver

  // Initialize the ibus data structure
  ibus_init(&ibus_data, numCh);

  // Set pin modes
  pinMode(PIN_ENC_CS, OUTPUT);
  pinMode(PIN_ENC_DATA, INPUT);
  pinMode(PIN_ENC_CLOCK, OUTPUT);
  pinMode(PIN_TOOTH1, OUTPUT);
  pinMode(PIN_TOOTH2, OUTPUT);

  // Set pin inital states
  digitalWrite(PIN_ENC_CLOCK, HIGH);
  digitalWrite(PIN_ENC_CS, LOW);
  digitalWrite(PIN_TOOTH1, HIGH);
  digitalWrite(PIN_TOOTH2, HIGH);
  
  // Uncomment for debugging
  Serial.begin(115200);   // Start serial interface to PC (debug)
  // Serial3.begin(115200);  // Start serial interface to Bluetooth
}

//===========//
// MAIN LOOP //
//===========//

void loop() {

  // Assume there is no new data from the reciever until proven otherwise
  bNewReceiverData = false;
  
  //======================================//
  // CHECK FOR NEW DATA FROM THE RECIEVER //
  //======================================//
  
  // If new data is available from the RC receiver...
  if (Serial2.available() > 0) {
    // Process it, and if it is the last byte of the current message...
    if (ibus_read(&ibus_data, chData, Serial2.read()) == 0) {
      bNewReceiverData = true;
      watchdogTimeout = millis() + 100;
    }
  }

  //=========================================//
  // RUN EVERY 20ms OR WHEN NEW DATA ARRIVES //
  //=========================================//

  // Run the control loop if...
  // 1. there is new data from the reciever, or
  // 2. more than 20ms have elapsed since the last time this loop was ran
  if (bNewReceiverData or millis() > lastLoopTime + 20){
    
    // Save the timestamp for when this loop started running
    lastLoopTime = millis();
    
    //=======================//
    // CHECK INVERSION STATE //
    //=======================//
    
    // Store the current state as the previous state
    prev_state = state;
    
    // Set the robot's inversion state based on SwitchA
    if (chData[6] > 1500) {
      state = INVERTED;
    }
    else{
      state = STANDARD;
    }
    
    //=======//
    // DRIVE //
    //=======//
    
    // STANDARD
    if (state == STANDARD){
      pwm_DriveMotorR = ConvertToDutyCycle(chData[1]);
      pwm_DriveMotorL = ConvertToDutyCycle(chData[2]);
    }
    
    // INVERTED
    else if (state == INVERTED){
      pwm_DriveMotorR = -ConvertToDutyCycle(chData[2]);
      pwm_DriveMotorL = -ConvertToDutyCycle(chData[1]);
    }
    
    // ERROR STATE
    else{
      pwm_DriveMotorR = 0;
      pwm_DriveMotorL = 0;
    }

    //===================//
    // READ WEDGE SENSOR //
    //===================//

    digitalWrite(PIN_ENC_CS, HIGH);
    digitalWrite(PIN_ENC_CS, LOW);
    int pv0 = 0;
    for (int i=0; i<10; i++) {
      digitalWrite(PIN_ENC_CLOCK, LOW);
      digitalWrite(PIN_ENC_CLOCK, HIGH);
      
      byte b = digitalRead(PIN_ENC_DATA) == HIGH ? 1 : 0;
      pv0 += b * pow(2, 10-(i+1));
    }
    
    for (int i=0; i<6; i++) {
      digitalWrite(PIN_ENC_CLOCK, LOW);
      digitalWrite(PIN_ENC_CLOCK, HIGH);
    }
    
    digitalWrite(PIN_ENC_CLOCK, LOW);
    digitalWrite(PIN_ENC_CLOCK, HIGH);
    
    //=======//
    // WEDGE //
    //=======//
    
    // Store the current wedge mode as the previous wedge mode
    prev_wedgeMode = wedgeMode;
    
    // Check wedge operational mode
    if (chData[7] > 1500) {wedgeMode = AUTOMATIC;}
    else {wedgeMode = MANUAL;}

    // Check for safety override mode
    if (chData[7] == 1500) {safety_override = true;}
    else {safety_override = false;}
    
    // MANUAL MODE
    if (wedgeMode == MANUAL){
    
      // Read Current Time and Wedge Position (still needed for safety limits)
      j = 0;
      t[j] = millis();
      pv[j] = pv0;

      // Initialize the PID variables for when automatic modes starts
      pid_int = 0;            // the integral term should be zero once automatic mode starts
      done_charging = false;  // pv averaging will have to recharge once automatic mode starts

      // If the wedge is being commanded up...
      if (ConvertToDutyCycle(chData[4]) > 0) {
        
        // Store the time the wedge was first commanded to move up
        if (move_wedge_t0 == 0) {
          move_wedge_t0 = millis();
        }
        
        // Slowly increase the wedge speed from zero to max speed over 500ms
        pwm_WedgeMotors =  32767 * manualWedgeModeSpeedScalingUp * min((double)(millis() - move_wedge_t0)/1000 + 0.5, 1);
      }
      // If the wedge is being commanded down...
      else if (ConvertToDutyCycle(chData[5]) > 0) {
        move_wedge_t0 = 0;
        pwm_WedgeMotors = -32767 * manualWedgeModeSpeedScalingDown;
      }
      // If there is no wedge command...
      else{
        move_wedge_t0 = 0;
        pwm_WedgeMotors = 0;
      }
      
      if (state == STANDARD){
        pwm_WedgeMotors =   1 * pwm_WedgeMotors;
      }
      else if (state == INVERTED){
        pwm_WedgeMotors = -1 * pwm_WedgeMotors;
      }
      else{
        pwm_WedgeMotors = 0;
      }
    }
  
    // AUTOMATIC MODE
    else if (wedgeMode == AUTOMATIC){

      // Store the previous set point
      sp_prev = sp;
      
      // STANDARD
      if (state == STANDARD){
        // Adjust Set Point (standard orientation)
        if (chData[4] > 1500) {sp = 600;}       // High
        else if (chData[5] > 1500) {sp = 490;}  // Low
        else {sp = 520;}                        // Mid
      }
      // INVERTED
      else if (state == INVERTED){
        // Adjust Set Point (inverted orientation)
        if (chData[4] > 1500) {sp = 500;}       // High (when the robot is upside down)
        else if (chData[5] > 1500) {sp = 590;}  // Low
        else {sp = 570;}                        // Mid
      }
      // ERROR CASE
      else {
        sp = 540;
      }
      
      // Read Current Time and Wedge Position
      j++; if (j >= ave_count){j=0; done_charging = true;}
      t[j] = millis();
      pv[j] = pv0;
      
      // If the digital encoder is disconnected, disable PID wedge control
      // Don't re-enable until the encoder has been connected for [ave_count] loop iterations
      if (pv[j] == 1020){
        done_charging = false;
        j = 0;
      }
      
      // Calculate Time Elapsed
      if (j==0) {dt = t[j] - t[ave_count-1];}
      else {dt = t[j] - t[j-1];}
      
      // Calculate average t and pv
      sumT = 0; sumPV = 0;
      for (i=0; i<ave_count; i++){
        sumT  += t[i];
        sumPV += pv[i];
      }
      aveT  = sumT  / ave_count;
      avePV = sumPV / ave_count;
      
      // Calculate best-fit derivative term
      sumRise = 0; sumRun = 0;
      for (i=0; i<ave_count; i++){
        devT  = t[i] - aveT;
        devPV = pv[i] - avePV;
        sumRise += devT * devPV;
        sumRun  += devT * devT;
      }
      pid_der = sumRise/sumRun;
      
      // Calculate the P & I terms
      pid_err = pv[j] - sp;
      if (abs(pid_err) < 3) {pid_err = 0;}
      pid_int = pid_int + pid_err * dt;

      // Cap the integral term so it can't drive the motors over 100%
      pid_int = min((double)32767/(-Ki), pid_int);
      pid_int = max((double)32767/(Ki), pid_int);

      // If the set point changed, disable the integral term for the specified amount of time
      if (sp != sp_prev or prev_wedgeMode != AUTOMATIC){
        pid_int = 0;
        pid_int_enable_time = millis() + 30;
      }
      else if (millis() < pid_int_enable_time) {
        pid_int = 0;
      }
      
      // Scale down the porpotional constant when the wedge is moving down
      if (state == STANDARD){
        if (pid_err > 0) {Kp2 = 0.75;}
        else {Kp2 = 1;}
      }
      else {
        if (pid_err < 0) {Kp2 = 0.75;}
        else {Kp2 = 1;}
      }
      
      // Calculate the output of the PID loop by summing and scaling the P,I,D terms
      pwm_WedgeMotors = Kp*Kp2*pid_err + Ki*pid_int + Kd*pid_der;
      //pwm_WedgeMotors = Kp*Kp2*pid_err + Ki*pid_int;
      
      // Only output to the motors if the pv array is full
      if (not done_charging){
        pwm_WedgeMotors = 0;  // Disable the wedge motors until the pv array fills up
        pid_int = 0;          // Don't allow integral term to charge when the PID loop isn't controlling the wedge motors
      }
    }
    
    //=======//
    // TEETH //
    //=======//
    
    // Store the old switch position
    prevToothSwitchPosition = currToothSwitchPosition;

    // Load the new switch position
    if (chData[9] < 1500) {currToothSwitchPosition = UP;}
    else {currToothSwitchPosition = DOWN;}
    
    // If it's the first time through the loop,
    // ensure prevToothSwitchPosition and currToothSwitchPosition have the same value.
    // This prevents the teeth from moving when the controller is first turned on.
    if (prevToothSwitchPosition == UNDEFINED) {prevToothSwitchPosition = currToothSwitchPosition;}
    
    // STANDARD
    if (state == STANDARD){
      // If the switch has been fliped from low to high...
      if (currToothSwitchPosition == UP and prevToothSwitchPosition == DOWN) {
        // Raise the teeth
        toothMotorCommand = RAISE;
        tsStateChangeTime = millis();
      }
      // If the switch has been fliped from high to low...
      else if (currToothSwitchPosition == DOWN and prevToothSwitchPosition == UP) {
        // Lower the teeth
        toothMotorCommand = LOWER;
        tsStateChangeTime = millis();
      }
      // If more than 250ms has elapsed since the last switch change...
      else if (millis() - tsStateChangeTime > 250) {
        // Stop the tooth motor
        toothMotorCommand = STOP;
      }
    }
  
    // INVERTED
    else if (state == INVERTED){
      // If you just entered inverted mode, start lowering the teeth (but for a longer duration than a normal pulse)
      if (prev_state != INVERTED){
        toothMotorCommand = LOWER;
        tsStateChangeTime = millis() + 150;
      }
      // If the teeth have been lowering for more than 100ms, stop them
      if (toothMotorCommand == LOWER and (long)millis() - (long)tsStateChangeTime > 100){
        toothMotorCommand = STOP;
        tsStateChangeTime = millis();
      }
      // If the teeth have been stopped for more than 1400ms, lower them
      if (toothMotorCommand == STOP and (long)millis() - (long)tsStateChangeTime > 1400){
        toothMotorCommand = LOWER;
        tsStateChangeTime = millis();
      }
    }
  
    // ERROR STATE
    else{
      // Stop the tooth motor
      toothMotorCommand = STOP;
    }
    
    //========//
    // LIMITS //
    //========//
    
    // Limit wedge motor speed to 100 percent
    pwm_WedgeMotors = min(32767, pwm_WedgeMotors);
    pwm_WedgeMotors = max(-32767, pwm_WedgeMotors);

    // Disable the wedge limits when safety override is true
    if (safety_override == false) {
      
      // Taper the max wedge speed down to zero near the ends of the range of travel
      if (pv[j] > 620 and pv[j] <= 650) {pwm_WedgeMotors = min(32767*(double)(650-pv[j])/30, pwm_WedgeMotors);}
      if (pv[j] < 490 and pv[j] >= 460) {pwm_WedgeMotors = max(32767*(double)(460-pv[j])/30, pwm_WedgeMotors);}
      
      // Disable the wedge motors at the top and bottom of the range of travel
      if (pv[j] > 650) {pwm_WedgeMotors = min(0, pwm_WedgeMotors);}
      if (pv[j] < 460) {pwm_WedgeMotors = max(0, pwm_WedgeMotors);}
      
    }
    
    //==========//
    // WATCHDOG //
    //==========//
    
    // If the receiver hasn't sent new data for over 100ms...
    if (millis() > watchdogTimeout){
      // Deactivate all motors
      pwm_DriveMotorL = 0;
      pwm_DriveMotorR = 0;
      pwm_WedgeMotors = 0;
      toothMotorCommand = STOP;
      pid_int = 0;
    }
    
    //====================================//
    // SEND COMMANDS TO MOTOR CONTROLLERS //
    //====================================//
    
    // Drive
    rc.DutyM1M2(rcDrive, pwm_DriveMotorR, pwm_DriveMotorL);
    
    // Wedge
    rc.DutyM1M2(rcWedge, (int)pwm_WedgeMotors, (int)pwm_WedgeMotors);
    
    // Teeth
    if (toothMotorCommand == RAISE){
      digitalWrite(PIN_TOOTH1, HIGH);
      digitalWrite(PIN_TOOTH2, LOW);
    }
    else if (toothMotorCommand == LOWER) {
      digitalWrite(PIN_TOOTH1, LOW);
      digitalWrite(PIN_TOOTH2, HIGH);
    }
    else{
      digitalWrite(PIN_TOOTH1, HIGH);
      digitalWrite(PIN_TOOTH2, HIGH);
    }
    
    //=======//
    // DEBUG //
    //=======//

    Serial.print(pv[j]); Serial.print("\t");
    Serial.print(sp); Serial.print("\t");
    Serial.println();
  }
}
//==================//
// END OF MAIN LOOP //
//==================//

//===========//
// FUNCTIONS //
//===========//

int ConvertToDutyCycle(uint16_t chData) {
  if (chData < 1000 or chData > 2000) return 64;  // If chData is not in expected range, stop motors
  int x = ((int)chData - 1500) * 65.534;          // Convert from 1000-2000 to (-32767)-(32767)
  return x;
}
