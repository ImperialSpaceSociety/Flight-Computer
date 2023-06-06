void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  while(Serial1.available() > 0)
  {
    char received = Serial1.read();
    if (received == '$') {
      Serial.println(received);
    } else {
      Serial.print(received);
    }
    delay(1);
  }
  delay(1);
}
