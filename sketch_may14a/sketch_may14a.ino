void setup() {
  Serial.begin(9600); // Start serial at 9600 baud
}

void loop() {
  if (Serial.available() > 0) {
    String received = Serial.readStringUntil('\n'); // Read line
    Serial.print("Arduino received: ");
    Serial.println(received); // Send response
  }
}
