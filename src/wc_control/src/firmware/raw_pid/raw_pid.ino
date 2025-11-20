/*
 * ROS2 Serial Mode MDDS30 PID Motor Control with Encoder Feedback
 * Pure Serial Communication - No ROS Dependencies
 * Converted from ROS1 functional code to pure serial mode
 * 
 * This code provides encoder feedback with PID control for MDDS30 motor driver
 * Compatible with ROS2 through serial communication
 * 
 * Set MDDS30 input mode to 0b10110100
 * 
 * Serial Commands:
 * - L<rpm>: Set left motor target RPM (e.g., L100 for 100 RPM)
 * - R<rpm>: Set right motor target RPM (e.g., R-50 for -50 RPM)
 * - S: Stop both motors
 * - P<value>: Set Kp gain (e.g., P2.5)
 * - I<value>: Set Ki gain (e.g., I0.1)
 * - D<value>: Set Kd gain (e.g., D0.05)
 * - T: Toggle PID tuning display
 * - V: Get current velocities and RPM
 * - G: Get current PID gains
 * 
 * Serial Output Format:
 * RPM_L:<left_rpm>,RPM_R:<right_rpm>,VEL_L:<left_vel>,VEL_R:<right_vel>
 * 
 * Maximum RPM: 168 (both directions)
 *
 * Hardware Connection:
 *   Arduino Mega       MDDS30
 *   GND -------------- GND
 *   6 ---------------- IN1
 *   4 ---------------- AN1
 *   5 ---------------- AN2
 *   7 ---------------- IN2
 *   53 --------------- Relay Control
 *
 *   Encoder Connections (User's setup):
 *   Pin 2  ----------- Left Encoder Channel A
 *   Pin 3  ----------- Left Encoder Channel B
 *   Pin 18 ----------- Right Encoder Channel A
 *   Pin 19 ----------- Right Encoder Channel B
 */

#include <Cytron_SmartDriveDuo.h>

// Motor driver pins
#define IN1 6 // Arduino pin 6 is connected to MDDS30 pin IN1.
#define AN1 4 // Arduino pin 4 is connected to MDDS30 pin AN1.
#define AN2 5 // Arduino pin 5 is connected to MDDS30 pin AN2.
#define IN2 7 // Arduino pin 7 is connected to MDDS30 pin IN2.

// Relay pin
int relayPin = 53;

// System constants - EXACT same as original
#define MAX_RPM 168
#define MAX_PWM 255
#define MIN_PWM -255
#define CONTROL_LOOP_TIME 50  // 50ms control loop (20Hz)
#define DISPLAY_TIME 200      // 200ms display update - SAME as original
#define WHEEL_RADIUS 0.3     // Wheel radius in meters (adjust for your robot)
#define ROS_TIMEOUT 1000      // ROS timeout in milliseconds - kept for reference

// Encoder pins (Corrected based on user's setup: 18,19 for RIGHT)
int encoderPin1L = 2;  // Left encoder channel A
int encoderPin2L = 3;  // Left encoder channel B
int encoderPin1R = 18; // Right encoder channel A
int encoderPin2R = 19; // Right encoder channel B

// Encoder variables - EXACT same as original
volatile int lastEncodedR = 0;
volatile long encoderValueR = 0;
volatile int lastEncodedL = 0;
volatile long encoderValueL = 0;

// Timing variables - EXACT same as original
unsigned long t;
unsigned long t_prev = 0;
unsigned long lastControlUpdate = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastRosCommand = 0; // Keep for potential future use

// RPM calculation variables - EXACT same as original
float pulseR, rpmR = 0;
float pulseR_prev = 0;
float dt;
float pulseL, rpmL = 0;
float pulseL_prev = 0;

// Velocity variables (m/s) - EXACT same as original
float velocityL = 0;  // Left wheel linear velocity
float velocityR = 0;  // Right wheel linear velocity

// PID variables for left motor - EXACT same as original
float targetRpmL = 0;
float errorL = 0, lastErrorL = 0;
float integralL = 0, derivativeL = 0;
float pidOutputL = 0, lastPidOutputL = 0;

// PID variables for right motor - EXACT same as original
float targetRpmR = 0;
float errorR = 0, lastErrorR = 0;
float integralR = 0, derivativeR = 0;
float pidOutputR = 0, lastPidOutputR = 0;

// PID gains (tunable) - EXACT same as original
float Kp = 0.2;   // Proportional gain
float Ki = 1.2;   // Integral gain
float Kd = 0.0;   // Derivative gain

// Anti-hunting features - EXACT same as original
float deadzone = 3.0;           // RPM deadzone to prevent hunting
float maxIntegral = 50.0;       // Maximum integral windup
float outputFilter = 0.8;       // Output smoothing filter
bool useOutputFilter = true;    // Enable/disable output filtering
float maxOutputChange = 5.0;   // Maximum PWM change per control cycle

// Motor control variables - EXACT same as original
Cytron_SmartDriveDuo smartDriveDuo30(PWM_INDEPENDENT, IN1, IN2, AN1, AN2);
int motorPwmL = 0, lastMotorPwmL = 0;
int motorPwmR = 0, lastMotorPwmR = 0;

// Control mode variables - EXACT same as original
bool showTuningInfo = true;
String inputString = "";
bool stringComplete = false;

// Motor direction inversion
#define INVERT_RIGHT true  // Set to true to invert right motor direction
#define INVERT_LEFT false  // Set to false if not needed

// Encoder interrupt functions - EXACT same as original
void updateEncoderR(){
  int MSB = digitalRead(encoderPin1R);
  int LSB = digitalRead(encoderPin2R);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedR << 2) | encoded;
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueR++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueR--;
  lastEncodedR = encoded;
}

void updateEncoderL(){
  int MSB = digitalRead(encoderPin1L);
  int LSB = digitalRead(encoderPin2L);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedL << 2) | encoded;
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueL++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueL--;
  lastEncodedL = encoded;
}

// Apply output smoothing to prevent sudden PWM changes - EXACT same as original
int smoothOutput(int newOutput, int lastOutput) {
  if (!useOutputFilter) return newOutput;
  
  int change = newOutput - lastOutput;
  change = constrain(change, -maxOutputChange, maxOutputChange);
  return lastOutput + change;
}

// PID calculation function - EXACT same as original
float calculatePID(float target, float current, float &error, float &lastError, float &integral, float &derivative, float dt) {
  error = target - current;
  integral += error * dt;
  
  derivative = (error - lastError) / dt;
  
  // Anti-windup: limit integral term - EXACT same as original
  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;
  
  float output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  
  return output;
}

// Convert RPM to PWM (simple mapping) - EXACT same as original
int rpmToPwm(float rpm) {
  // Map RPM to PWM range (-255 to 255)
  return constrain(map(rpm, -MAX_RPM, MAX_RPM, MIN_PWM, MAX_PWM), MIN_PWM, MAX_PWM);
}

// Convert RPM to velocity (m/s) - EXACT same as original
float rpmToVelocity(float rpm) {
  return (rpm * 2.0 * PI * WHEEL_RADIUS) / 60.0;
}

// Process serial commands - Enhanced from original
void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  if (command.startsWith("L")) {
    // Set left motor target RPM
    targetRpmL = constrain(command.substring(1).toFloat(), -MAX_RPM, MAX_RPM);
    Serial.print("Left target RPM: ");
    Serial.println(targetRpmL);
    lastRosCommand = millis(); // Update command timestamp
  }
  else if (command.startsWith("R")) {
    // Set right motor target RPM
    targetRpmR = constrain(command.substring(1).toFloat(), -MAX_RPM, MAX_RPM);
    Serial.print("Right target RPM: ");
    Serial.println(targetRpmR);
    lastRosCommand = millis(); // Update command timestamp
  }
  else if (command.startsWith("P")) {
    // Set Kp gain
    Kp = command.substring(1).toFloat();
    Serial.print("Kp set to: ");
    Serial.println(Kp);
  }
  else if (command.startsWith("I")) {
    // Set Ki gain
    Ki = command.substring(1).toFloat();
    Serial.print("Ki set to: ");
    Serial.println(Ki);
  }
  else if (command.startsWith("D")) {
    // Set Kd gain
    Kd = command.substring(1).toFloat();
    Serial.print("Kd set to: ");
    Serial.println(Kd);
  }
  else if (command == "T") {
    // Toggle tuning display
    showTuningInfo = !showTuningInfo;
    Serial.print("Tuning display: ");
    Serial.println(showTuningInfo ? "ON" : "OFF");
  }
  else if (command == "S") {
    // Emergency stop
    targetRpmL = 0;
    targetRpmR = 0;
    integralL = 0;
    integralR = 0;
    Serial.println("Emergency stop - both motors stopped");
  }
  else if (command == "V") {
    // Get current velocities - NEW feature for ROS2 compatibility
    Serial.print("VELOCITY_DATA:");
    Serial.print("L_RPM="); Serial.print(rpmL, 2);
    Serial.print(",R_RPM="); Serial.print(rpmR, 2);
    Serial.print(",L_VEL="); Serial.print(velocityL, 3);
    Serial.print(",R_VEL="); Serial.println(velocityR, 3);
  }
  else if (command == "G") {
    // Get current PID gains - NEW feature
    Serial.print("PID_GAINS:");
    Serial.print("Kp="); Serial.print(Kp);
    Serial.print(",Ki="); Serial.print(Ki);
    Serial.print(",Kd="); Serial.println(Kd);
  }
  else {
    Serial.println("Available commands:");
    Serial.println("  L<rpm>  - Set left motor target RPM (e.g., L100)");
    Serial.println("  R<rpm>  - Set right motor target RPM (e.g., R-50)");
    Serial.println("  P<val>  - Set Kp gain (e.g., P2.5)");
    Serial.println("  I<val>  - Set Ki gain (e.g., I0.1)");
    Serial.println("  D<val>  - Set Kd gain (e.g., D0.05)");
    Serial.println("  T       - Toggle PID tuning display");
    Serial.println("  S       - Emergency stop");
    Serial.println("  V       - Get current velocities and RPM");
    Serial.println("  G       - Get current PID gains");
  }
}

void setup() {
  // Initialize relay pin
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Relay OFF initially
  
  // Initialize LED pin
  pinMode(13, OUTPUT);
  
  // Initialize serial communication - increased baud rate for better performance
  Serial.begin(115200);
  inputString.reserve(50);
  
  // Initialize encoder pins - EXACT same as original
  pinMode(encoderPin1R, INPUT);
  pinMode(encoderPin2R, INPUT);
  pinMode(encoderPin1L, INPUT);
  pinMode(encoderPin2L, INPUT);
  
  // Enable pullup resistors - EXACT same as original
  digitalWrite(encoderPin1R, HIGH);
  digitalWrite(encoderPin2R, HIGH);
  digitalWrite(encoderPin1L, HIGH);
  digitalWrite(encoderPin2L, HIGH);
  
  // Attach interrupts for encoder feedback - EXACT same as original
  attachInterrupt(digitalPinToInterrupt(encoderPin1R), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2R), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin1L), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2L), updateEncoderL, CHANGE);
  
  // Wait 5 seconds before turning on relay
  Serial.println("MDDS30 Serial Mode with Encoder Feedback Ready");
  Serial.println("Waiting 5 seconds before powering motor driver...");
  delay(5000);
  
  // Turn on relay to power the driver
  digitalWrite(relayPin, LOW); // Turn relay ON
  Serial.println("Relay ON - Motor driver powered");
  Serial.print("Relay status (0=ON): "); Serial.println(digitalRead(relayPin));
  
  // Initial LED indication - EXACT same as original
  digitalWrite(13, HIGH);
  delay(2000);
  digitalWrite(13, LOW);
  
  // Initialize timing - EXACT same as original
  t_prev = micros();
  lastControlUpdate = millis();
  lastDisplayUpdate = millis();
  lastRosCommand = millis();
  
  Serial.println("ROS2 Serial Mode MDDS30 Motor Control Ready");
  Serial.println("Max RPM: 168 (both directions)");
  Serial.println("Serial commands available for control and tuning:");
  Serial.println("  L<rpm>  - Set left motor target RPM (e.g., L100)");
  Serial.println("  R<rpm>  - Set right motor target RPM (e.g., R-50)");
  Serial.println("  P<val>  - Set Kp gain (e.g., P2.5)");
  Serial.println("  I<val>  - Set Ki gain (e.g., I0.1)");
  Serial.println("  D<val>  - Set Kd gain (e.g., D0.05)");
  Serial.println("  T       - Toggle PID tuning display");
  Serial.println("  S       - Emergency stop");
  Serial.println("Current PID gains: Kp=" + String(Kp) + ", Ki=" + String(Ki) + ", Kd=" + String(Kd));
}

void loop() {
  unsigned long currentTime = millis();
  
  // Handle serial input - same as your current code
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        processCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += inChar;
    }
  }
  
  // Control loop - EXACT same structure as original (but always runs for responsiveness)
  // if (currentTime - lastControlUpdate >= CONTROL_LOOP_TIME) {
    lastControlUpdate = currentTime;
    
    // Calculate RPM - EXACT same as original
    noInterrupts();
    t = micros();
    dt = (t - t_prev) / 1000000.0;
    
    if (dt > 0) {
      pulseR = encoderValueR / (4 * 2500.0);
      rpmR = ((pulseR - pulseR_prev) / dt) * 60;
      pulseL = encoderValueL / (4 * 2500.0);
      rpmL = ((pulseL - pulseL_prev) / dt) * 60;
      
      pulseR_prev = pulseR;
      pulseL_prev = pulseL;
      t_prev = t;
    }
    interrupts();
    
    // Convert RPM to velocity for ROS publishing - EXACT same as original
    velocityL = rpmToVelocity(rpmL);
    velocityR = rpmToVelocity(rpmR);
    
    // Calculate PID for both motors - EXACT same as original
    pidOutputL = calculatePID(targetRpmL, rpmL, errorL, lastErrorL, integralL, derivativeL, dt);
    pidOutputR = calculatePID(targetRpmR, rpmR, errorR, lastErrorR, integralR, derivativeR, dt);
    
    // Convert PID output to PWM values with constraints - EXACT same as original
    int newPwmL = constrain(pidOutputL, MIN_PWM, MAX_PWM);
    int newPwmR = constrain(pidOutputR, MIN_PWM, MAX_PWM);
    
    // Apply output smoothing - EXACT same as original
    motorPwmL = smoothOutput(newPwmL, lastMotorPwmL);
    motorPwmR = smoothOutput(newPwmR, lastMotorPwmR);
    
    // Apply motor direction inversion
    int appliedPwmL = INVERT_LEFT ? -motorPwmL : motorPwmL;
    int appliedPwmR = INVERT_RIGHT ? -motorPwmR : motorPwmR;
    
    // Apply motor control - EXACT same as original
    smartDriveDuo30.control(appliedPwmL, appliedPwmR);
    
    // Store last values - EXACT same as original
    lastMotorPwmL = motorPwmL;
    lastMotorPwmR = motorPwmR;
    
    // Update LED status - EXACT same as original
    digitalWrite(13, (targetRpmL != 0 || targetRpmR != 0) ? HIGH : LOW);
  // }

  // Display update - runs every DISPLAY_TIME ms - EXACT same timing as original
  if (currentTime - lastDisplayUpdate >= DISPLAY_TIME) {
    lastDisplayUpdate = currentTime;
    
    // Always send structured data for ROS2 compatibility
    Serial.print("RPM_L:"); Serial.print(rpmL, 1);
    Serial.print(",RPM_R:"); Serial.print(rpmR, 1);
    Serial.print(",VEL_L:"); Serial.print(velocityL, 3);
    Serial.print(",VEL_R:"); Serial.println(velocityR, 3);
    
    if (showTuningInfo) {
      // Detailed tuning information - EXACT same as original
      Serial.print("L: T="); Serial.print(targetRpmL, 1);
      Serial.print(" A="); Serial.print(rpmL, 1);
      Serial.print(" E="); Serial.print(errorL, 1);
      Serial.print(" PWM="); Serial.print(motorPwmL);
      Serial.print(" | R: T="); Serial.print(targetRpmR, 1);
      Serial.print(" A="); Serial.print(rpmR, 1);
      Serial.print(" E="); Serial.print(errorR, 1);
      Serial.print(" PWM="); Serial.println(motorPwmR);
    }
  }
}