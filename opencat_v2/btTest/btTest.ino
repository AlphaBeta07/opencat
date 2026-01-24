void setup() {
  Serial.begin(9600);
  Serial.println("BT Test Ready");
}

void loop() {
  if(Serial.available()) {
    char c = Serial.read();
    Serial.print("CMD: ");
    Serial.println(c);
  }
}
