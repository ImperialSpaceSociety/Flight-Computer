#include<SPI.h>

// Register values
#define REG_CHIP_MODE 0x01
#define REG_OCP 0x0B
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_FREQ_HOP 0x22
#define REG_SYNC_WORD 0x39
#define REG_CHIP_ID 0x42

SPISettings settings = SPISettings(2000000, MSBFIRST, SPI_MODE0);
int CS_PIN = 9;
int IRQ = 4;
int GPIO = 5;
int RST = 6;
int SX1272_CHIP_ID = 0x22;

uint8_t WRITE_COMMAND = 0x80;
uint8_t MODE_REGISTER = 0x01;
uint8_t DEFAULT_LORA_SYNC_WORD = 0x12;
uint16_t DEFAULT_PREAMBLE_LENGTH = 0x0008;

void setup() {
  Serial.begin(9600);

  pinMode(IRQ, INPUT);
  pinMode(GPIO, INPUT);
  pinMode(RST, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  SPI.begin();
}

void loop() {
  Serial.println("--------------- START ---------------");
  uint8_t receivedVal = SPIRegRead(REG_CHIP_ID);
  Serial.print("Reading chip ID: ");
  Serial.println(receivedVal);
  delay(100);

  receivedVal = SPIRegRead(REG_CHIP_MODE);
  Serial.print("Reading chip mode: ");
  Serial.println(receivedVal);
  delay(100);

  receivedVal = SPIRegRead(REG_FREQ_HOP);
  Serial.print("Reading frequency hop: ");
  Serial.println(receivedVal);
  delay(100);

  uint8_t valueToWrite = receivedVal == 0 ? 5 : 0;

  receivedVal = SPIRegWrite(REG_FREQ_HOP, valueToWrite);
  Serial.print("Writing frequency hop: ");
  Serial.println(receivedVal);
  delay(100);

  Serial.println("Setting FSK");
  setFSKMode();
  receivedVal = SPIRegRead(REG_CHIP_MODE);
  Serial.print("Reading chip mode: ");
  Serial.println(receivedVal);
  delay(100);

  Serial.println("Setting LORA");
  setLORAMode();
  receivedVal = SPIRegRead(REG_CHIP_MODE);
  Serial.print("Reading chip mode: ");
  Serial.println(receivedVal);
  delay(100);

  Serial.println("Setting STANDBY mode");
  setSTANDBYMode();
  delay(10);
  receivedVal = SPIRegRead(REG_CHIP_MODE);
  Serial.print("Reading chip mode (should be 129): ");
  Serial.println(receivedVal);
  delay(100);

  Serial.println("Setting LORA sync word");
  setRadioSyncWord(DEFAULT_LORA_SYNC_WORD);
  delay(10);
  receivedVal = SPIRegRead(REG_SYNC_WORD);
  Serial.print("Reading LORA sync word (should be 18): ");
  Serial.println(receivedVal);
  delay(100);

  Serial.println("Setting the current limit");
  setCurrentLimit(60);
  delay(10);
  receivedVal = SPIRegRead(REG_OCP);
  Serial.print("Reading over current protection (should be 35): ");
  Serial.println(receivedVal);
  delay(100);

  Serial.println("Setting the Preamble length");
  setPreambleLength(DEFAULT_PREAMBLE_LENGTH);
  delay(10);
  receivedVal = SPIRegRead(REG_PREAMBLE_MSB);
  Serial.print("Reading preamble length (MSB): ");
  Serial.print(receivedVal);
  receivedVal = SPIRegRead(REG_PREAMBLE_LSB);
  Serial.print(" preamble length (LSB): ");
  Serial.println(receivedVal);
  delay(100);
  
  Serial.println("--------------- END ---------------");
}

void setLORAMode() {
  uint8_t currentMode = SPIRegRead(REG_CHIP_MODE);
  uint8_t currentModeSleep = currentMode & 0xF8;
  uint8_t receivedVal = SPIRegWrite(REG_CHIP_MODE, currentModeSleep);
  receivedVal = SPIRegWrite(REG_CHIP_MODE, 0x80);
}

void setFSKMode() {
  uint8_t currentMode = SPIRegRead(REG_CHIP_MODE);
  uint8_t currentModeSleep = currentMode & 0xF8;
  uint8_t receivedVal = SPIRegWrite(REG_CHIP_MODE, currentModeSleep);
  receivedVal = SPIRegWrite(REG_CHIP_MODE, 0x0);
}

void setSTANDBYMode() {
  uint8_t currentMode = SPIRegRead(REG_CHIP_MODE);
  uint8_t currentModeStandby = currentMode & 0xF8 | 0x01;
  uint8_t receivedVal = SPIRegWrite(REG_CHIP_MODE, currentModeStandby);
}

void setRadioSyncWord(uint8_t syncWord) {
  uint8_t receivedVal = SPIRegWrite(REG_SYNC_WORD, syncWord);
}

void setCurrentLimit(uint8_t currentLimit) {
  setSTANDBYMode();

  if(!(((currentLimit >= 45) && (currentLimit <= 240)) || (currentLimit == 0))) {
    Serial.println("The current limit is not within the allowed threshold!");
  } else {
    if(currentLimit <= 120) {
      uint8_t raw = (currentLimit - 45) / 5;
      SPIRegWrite(REG_OCP, 0x20 | raw);
    } else if(currentLimit <= 240) {
      uint8_t raw = (currentLimit + 30) / 10;
      SPIRegWrite(REG_OCP, 0x20 | raw);
    }
  }
}

void setPreambleLength(uint16_t preambleLength) {

}

uint8_t SPIRegRead(uint8_t reg) {
  SPI.beginTransaction(settings);
  digitalWrite(CS_PIN, LOW);

  uint8_t receivedVal = SPI.transfer(reg);
  receivedVal = SPI.transfer(0);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return receivedVal;
}

uint8_t SPIRegWrite(uint8_t reg, uint8_t val) {
  // TODO: maybe add write 
  SPI.beginTransaction(settings);
  digitalWrite(CS_PIN, LOW);

  uint8_t writeCommand = WRITE_COMMAND | reg;
  uint8_t receivedVal = SPI.transfer(writeCommand);
  receivedVal = SPI.transfer(val);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return receivedVal;
}
