#ifndef __I2C_H__
#define __I2C_H__

// === INCLUDES ===
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

// === FUNCTIONS ===
void write8(uint8_t addr, uint8_t reg, uint8_t val);
uint8_t read8(uint8_t addr, uint8_t reg);
uint16_t read16(uint8_t addr, uint8_t reg);
uint16_t read16Be(uint8_t addr, uint8_t reg);
uint32_t read24(uint8_t addr, uint8_t reg);
void read(uint8_t addr, uint8_t reg, uint8_t* buf, uint16_t len);

#endif 
