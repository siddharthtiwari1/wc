int relayPin = 53;

void setup() {
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);  // Relay OFF initially
  
  delay(5000);  // Keep relay OFF for 5 seconds
  
  digitalWrite(relayPin, LOW);   // Turn relay ON after 5 seconds
}

void loop() {
  // Relay stays ON (LOW signal)
  // Your other code can go here
}


