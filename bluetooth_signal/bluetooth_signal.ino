void setup() {
  Serial.begin(9600);   // HC-05 default baud
}

void loop() {
  if (Serial.available()) {
    char incoming = Serial.read();
    Serial.print("Received: ");
    Serial.println(incoming);
  }
}
// Stand =  A
// Sit =  M
// Lay =  D
// Sleep =  C