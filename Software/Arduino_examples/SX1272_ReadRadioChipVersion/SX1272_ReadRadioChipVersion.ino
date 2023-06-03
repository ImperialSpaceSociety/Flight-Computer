#include<SPI.h>

SPISettings settings = SPISettings(2000000, MSBFIRST, SPI_MODE0);
int CS_PIN = 9;
int IRQ = 4;
int GPIO = 5;
int RST = 6;
int SX1272_CHIP_ID = 0x22;

void setup() {
  Serial.begin(9600);

  pinMode(IRQ, INPUT);
  pinMode(GPIO, INPUT);
  pinMode(RST, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  SPI.begin();
}

void loop() {
  SPI.beginTransaction(settings);
  digitalWrite(CS_PIN, LOW);
  
  uint8_t receivedVal = SPI.transfer(66);
  Serial.print("Received value 1: ");
  Serial.print(receivedVal);
  receivedVal = SPI.transfer(0);
  Serial.print(" Received value 2: ");
  Serial.println(receivedVal);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  if (receivedVal == SX1272_CHIP_ID) {
    Serial.println("SX1272 chip detected!");
  }
  
  delay(100);
}
