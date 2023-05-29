#include <RadioLib.h>
#include <SPI.h>

SPISettings settings = SPISettings(2000000, MSBFIRST, SPI_MODE0);
int CS_PIN = 9;
int IRQ = 4;
int GPIO = 5;
int RST = 6;

void setup() {
  SPI.begin();
  Serial.begin(9600);

  SX1272 radio = new Module(9, 4, 5, 6);

  int state = radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX127X_SYNC_WORD, 17);
  //radio.setOutputPower(20, false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  pinMode(IRQ, INPUT);
  pinMode(GPIO, INPUT);
  pinMode(RST, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
}

void loop() {
  //transmit();
  SPI.beginTransaction(settings);
  digitalWrite(CS_PIN, LOW);
  
  uint8_t receivedVal = SPI.transfer(29);
  Serial.print("Received value 1: ");
  Serial.print(receivedVal);
  receivedVal = SPI.transfer(0);
  Serial.print(" Received value 2: ");
  Serial.println(receivedVal);
  /*Serial.print(" Received value 3: ");
  Serial.println(receivedVal);*/
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  delay(100);
}
