#include<SPI.h>

// Register values
#define REG_CHIP_MODE 0x01
#define REG_CHIP_ID 0x42

SPISettings settings = SPISettings(2000000, MSBFIRST, SPI_MODE0);
int CS_PIN = 9;
int IRQ = 4;
int GPIO = 5;
int RST = 6;
int SX1272_CHIP_ID = 0x22;

uint8_t WRITE_COMMAND = 0x80;
uint8_t MODE_REGISTER = 0x01;
bool isZero = false;

void setup() {
  Serial.begin(9600);

  pinMode(IRQ, INPUT);
  pinMode(GPIO, INPUT);
  pinMode(RST, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  SPI.begin();
}

void loop() {
  Serial.println("Reading chip ID");

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
  
  delay(100);

  Serial.println("Reading chip mode");
  receivedVal = SPIRegRead(REG_CHIP_MODE);
  SPI.beginTransaction(settings);
  digitalWrite(CS_PIN, LOW);
  
  receivedVal = SPI.transfer(1);
  Serial.print("Received value 1: ");
  Serial.print(receivedVal);
  receivedVal = SPI.transfer(0);
  Serial.print(" Received value 2: ");
  Serial.println(receivedVal);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  
  delay(100);

  Serial.println("Setting the chip mode");

  SPI.beginTransaction(settings);
  digitalWrite(CS_PIN, LOW);
  
  receivedVal = SPI.transfer(129);
  Serial.print("Received value 1: ");
  Serial.print(receivedVal);
  receivedVal = SPI.transfer(1);
  Serial.print(" Received value 2: ");
  Serial.println(receivedVal);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  delay(100);

  Serial.println("Reading config");

  uint8_t configRegister = 36;

  SPI.beginTransaction(settings);
  digitalWrite(CS_PIN, LOW);
  
  receivedVal = SPI.transfer(configRegister);
  Serial.print("Received value 1: ");
  Serial.print(receivedVal);
  receivedVal = SPI.transfer(0);
  Serial.print(" Received value 2: ");
  Serial.println(receivedVal);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  delay(100);

  Serial.println("Setting config");

  SPI.beginTransaction(settings);
  digitalWrite(CS_PIN, LOW);

  uint8_t commandToBeSent = WRITE_COMMAND | configRegister;
  uint8_t valueToWrite = isZero ? 0 : 5;
  isZero = !isZero;

  Serial.print("Command to be sent ");
  Serial.println(commandToBeSent);
  
  receivedVal = SPI.transfer(commandToBeSent);
  Serial.print("Received value 1: ");
  Serial.print(receivedVal);
  receivedVal = SPI.transfer(valueToWrite);
  Serial.print(" Received value 2: ");
  Serial.println(receivedVal);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  delay(100);
}
