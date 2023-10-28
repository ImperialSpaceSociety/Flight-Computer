// === INCLUDES ===
#include "i2c.h"

// === FUNCTIONS ===
void write8(uint8_t addr, uint8_t reg, uint8_t val) {
	uint8_t data[2] = {reg, val};
	HAL_I2C_Master_Transmit(&hi2c1, addr << 1, data, 2, HAL_MAX_DELAY);
}

uint8_t read8(uint8_t addr, uint8_t reg) {
	uint8_t data = 0;
	HAL_I2C_Master_Transmit(&hi2c1, addr << 1, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, addr << 1, &data, 1, HAL_MAX_DELAY);
	return data;
}

uint16_t read16(uint8_t addr, uint8_t reg) {
	uint8_t data[2] = {0};
	HAL_I2C_Master_Transmit(&hi2c1, addr << 1, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, addr << 1, data, 2, HAL_MAX_DELAY);
	return (data[1] | (data[0] << 8));
}

uint16_t read16Be(uint8_t addr, uint8_t reg) {
	uint8_t data[2] = {0};
	HAL_I2C_Master_Transmit(&hi2c1, addr << 1, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, addr << 1, data, 2, HAL_MAX_DELAY);
	return (data[0] | (data[1] << 8));
}

uint32_t read24(uint8_t addr, uint8_t reg) {
	uint8_t data[3] = {0};
	HAL_I2C_Master_Transmit(&hi2c1, addr << 1, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, addr << 1, data, 3, HAL_MAX_DELAY);
	return (data[2] | (data[1] << 8) | (data[0] << 16));
}

void read(uint8_t addr, uint8_t reg, uint8_t* buf, uint16_t len) {
	HAL_I2C_Master_Transmit(&hi2c1, addr << 1, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, addr << 1, buf, len, HAL_MAX_DELAY);
}
