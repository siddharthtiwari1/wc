// Encoder pins
int encoderPin1R = 18;  
int encoderPin2R = 19;  
int encoderPin1L = 3; 
int encoderPin2L = 2; 

// Relay Pin
int relayPin = 53;

// Motor driver pins
#define IN1 7 
#define AN1 5 
#define AN2 4 
#define IN2 6 

#include <Cytron_SmartDriveDuo.h>

Cytron_SmartDriveDuo smartDriveDuo(PWM_INDEPENDENT, IN1, IN2, AN1, AN2);

signed int speedLeft = 0;
signed int speedRight = 0;

// Encoder variables for Left
volatile int lastEncodedL = 0;
volatile long encoderValueL = 0;

// Encoder variables for Right
volatile int lastEncodedR = 0;
volatile long encoderValueR = 0;

// For RPM calculation
const long CPR = 10000; // Counts Per Revolution
const float MAX_RPM = 168.0;
unsigned long lastTimePID = 0;
long lastEncoderL = 0;
long lastEncoderR = 0;

// Target RPM
float targetRPML = 0.0;
float targetRPMR = 0.0;

// PID constants (tune these based on testing)
double Kp = 0.4;  // Proportional gain
double Ki = 3.9;  // Integral gain
double Kd = 0.01; // Derivative gain

// PID variables for Left
double integralL = 0.0;
double previousErrorL = 0.0;
float previousRPML = 0.0;
float currentRPML = 0.0;

// PID variables for Right
double integralR = 0.0;
double previousErrorR = 0.0;
float previousRPMR = 0.0;
float currentRPMR = 0.0;

// PID update interval (ms) - increased frequency for robustness
const unsigned long PID_INTERVAL = 50;

// Print interval (ms)
const unsigned long PRINT_INTERVAL = 1000;
unsigned long lastPrintTime = 0;

void updateEncoderL(){
  int MSB = digitalRead(encoderPin1L);
  int LSB = digitalRead(encoderPin2L);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedL << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueL ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueL --;

  lastEncodedL = encoded;
}

void updateEncoderR(){
  int MSB = digitalRead(encoderPin1R);
  int LSB = digitalRead(encoderPin2R);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedR << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueR ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueR --;

  lastEncodedR = encoded;
}

void setup() {
  Serial.begin(115200); // High baud rate

  // Initialize relay pin
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);
  delay(5000);
  digitalWrite(relayPin, LOW);

  // Encoder pins for Left
  pinMode(encoderPin1L, INPUT);
  pinMode(encoderPin2L, INPUT);
  digitalWrite(encoderPin1L, HIGH); // turn pullup resistor on
  digitalWrite(encoderPin2L, HIGH); // turn pullup resistor on

  // Attach interrupts for Left
  attachInterrupt(digitalPinToInterrupt(encoderPin1L), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2L), updateEncoderL, CHANGE);

  // Encoder pins for Right
  pinMode(encoderPin1R, INPUT);
  pinMode(encoderPin2R, INPUT);
  digitalWrite(encoderPin1R, HIGH); // turn pullup resistor on
  digitalWrite(encoderPin2R, HIGH); // turn pullup resistor on

  // Attach interrupts for Right
  attachInterrupt(digitalPinToInterrupt(encoderPin1R), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2R), updateEncoderR, CHANGE);

  // Initialize timings and previous RPM
  lastTimePID = millis();
  lastEncoderL = 0;
  lastEncoderR = 0;
  previousRPML = 0.0;
  previousRPMR = 0.0;
  lastPrintTime = millis();
}

void loop() {
  // Read serial commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.replace(" ", ""); // Remove spaces for flexibility (e.g., "L 100" becomes "L100")

    if (cmd.startsWith("L") || cmd.startsWith("l")) {
      String valStr = cmd.substring(1);
      targetRPML = valStr.toFloat();
      if (targetRPML > MAX_RPM) targetRPML = MAX_RPM;
      if (targetRPML < -MAX_RPM) targetRPML = -MAX_RPM;
      if (targetRPML == 0) {
        speedLeft = 0;
        integralL = 0.0;
      }
      integralL = 0.0; // Reset integral on target change to avoid windup
    } else if (cmd.startsWith("R") || cmd.startsWith("r")) {
      String valStr = cmd.substring(1);
      targetRPMR = valStr.toFloat();
      if (targetRPMR > MAX_RPM) targetRPMR = MAX_RPM;
      if (targetRPMR < -MAX_RPM) targetRPMR = -MAX_RPM;
      if (targetRPMR == 0) {
        speedRight = 0;
        integralR = 0.0;
      }
      integralR = 0.0; // Reset integral on target change
    }
  }

  // PID and RPM calculation
  unsigned long now = millis();

  if (now - lastTimePID >= PID_INTERVAL) {
    float deltaT = max(0.001f, (now - lastTimePID) / 1000.0f); // Safeguard against zero deltaT

    // Left wheel
    long deltaCountsL = encoderValueL - lastEncoderL;
    float rpmL = - (deltaCountsL / (float)CPR) * (60.0 / deltaT); // Inverted sign for polarity match
    currentRPML = rpmL;

    // Right wheel
    long deltaCountsR = encoderValueR - lastEncoderR;
    float rpmR = (deltaCountsR / (float)CPR) * (60.0 / deltaT); // Removed negation for polarity match
    currentRPMR = rpmR;

    if (targetRPML != 0) {
      double errorL = targetRPML - rpmL;
      integralL += errorL * deltaT;
      double derivativeL = - (rpmL - previousRPML) / deltaT; // Derivative on PV for no setpoint kick
      double ffL = (targetRPML / MAX_RPM) * 100.0;
      double pidOutputL = Kp * errorL + Ki * integralL + Kd * derivativeL;
      double outputL = ffL + pidOutputL;
      speedLeft = constrain(outputL, -100, 100);

      // Back-calculation anti-windup
      if (speedLeft != outputL) {
        integralL = (speedLeft - ffL - Kp * errorL - Kd * derivativeL) / Ki;
      }

      previousErrorL = errorL;
      previousRPML = rpmL;
    }

    if (targetRPMR != 0) {
      double errorR = targetRPMR - rpmR;
      integralR += errorR * deltaT;
      double derivativeR = - (rpmR - previousRPMR) / deltaT; // Derivative on PV
      double ffR = (targetRPMR / MAX_RPM) * 100.0;
      double pidOutputR = Kp * errorR + Ki * integralR + Kd * derivativeR;
      double outputR = ffR + pidOutputR;
      speedRight = constrain(outputR, -100, 100);

      // Back-calculation anti-windup
      if (speedRight != outputR) {
        integralR = (speedRight - ffR - Kp * errorR - Kd * derivativeR) / Ki;
      }

      previousErrorR = errorR;
      previousRPMR = rpmR;
    }

    smartDriveDuo.control(speedRight, -speedLeft);

    lastEncoderL = encoderValueL;
    lastEncoderR = encoderValueR;
    lastTimePID = now;
  }

  // Print RPM every PRINT_INTERVAL
  if (now - lastPrintTime >= PRINT_INTERVAL) {
    Serial.print("Left RPM: ");
    Serial.print(currentRPML);
    Serial.print(" (Target: ");
    Serial.print(targetRPML);
    Serial.println(")");

    Serial.print("Right RPM: ");
    Serial.print(currentRPMR);
    Serial.print(" (Target: ");
    Serial.print(targetRPMR);
    Serial.println(")");

    lastPrintTime = now;
  }
}