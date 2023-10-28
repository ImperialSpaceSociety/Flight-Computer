#include "BMI088.h"

float accRange;
float gyroRange;

void BMI088Init(void) {
	BMI088SetAccRange(RANGE_6G);
	BMI088SetAccRate(ODR_100);
	BMI088SetAccPower(ACC_ACTIVE);

	BMI088SetGyroRange(RANGE_2000);
	BMI088SetGyroRate(ODR_2000_BW_532);
	BMI088SetGyroPower(GYRO_NORMAL);
}

bool BMI088Check(void) {
	return (
		(read8(BMI088_ACC_I2C_ADDR, BMI088_ACC_CHIP_REG) == BMI088_ACC_CHIP_ID) &&
		(read8(BMI088_GYRO_I2C_ADDR, BMI088_GYRO_CHIP_REG) == BMI088_GYRO_CHIP_ID)
	);
}

void BMI088ResetAcc(void) {
	write8(BMI088_ACC_I2C_ADDR, BMI088_ACC_SOFT_RESET, 0xB6);
}

void BMI088ResetGyro(void) {
	write8(BMI088_GYRO_I2C_ADDR, BMI088_GYRO_SOFT_RESET, 0xB6);
}

void BMI088SetAccPower(acc_power_type_t mode) {
	if (mode == ACC_ACTIVE) {
		write8(BMI088_ACC_I2C_ADDR, BMI088_ACC_PWR_CTRl, 0x04);
		write8(BMI088_ACC_I2C_ADDR, BMI088_ACC_PWR_CONF, 0x00);
	} else if (mode == ACC_SUSPEND) {
		write8(BMI088_ACC_I2C_ADDR, BMI088_ACC_PWR_CONF, 0x03);
		write8(BMI088_ACC_I2C_ADDR, BMI088_ACC_PWR_CTRl, 0x00);
	}
}

void BMI088SetGyroPower(gyro_power_type_t mode) {
	if (mode == GYRO_NORMAL) {
		write8(BMI088_GYRO_I2C_ADDR, BMI088_GYRO_LPM_1, (uint8_t)GYRO_NORMAL);
	} else if (mode == GYRO_SUSPEND) {
		write8(BMI088_GYRO_I2C_ADDR, BMI088_GYRO_LPM_1, (uint8_t)GYRO_SUSPEND);
	} else if (mode == GYRO_DEEP_SUSPEND) {
		write8(BMI088_GYRO_I2C_ADDR, BMI088_GYRO_LPM_1, (uint8_t)GYRO_DEEP_SUSPEND);
	}
}

void BMI088SetAccRange(acc_scale_type_t range) {
	if (range == RANGE_3G) {
		accRange = 3000;
	} else if (range == RANGE_6G) {
		accRange = 6000;
	} else if (range == RANGE_12G) {
		accRange = 12000;
	} else if (range == RANGE_24G) {
		accRange = 24000;
	}

	write8(BMI088_ACC_I2C_ADDR, BMI088_ACC_RANGE, (uint8_t)range);
}

void BMI088SetAccRate(acc_odr_type_t odr) {
	uint8_t data = 0;

	data = read8(BMI088_ACC_I2C_ADDR, BMI088_ACC_CONF);
	data = data & 0xf0;
	data = data | (uint8_t)odr;

	write8(BMI088_ACC_I2C_ADDR, BMI088_ACC_CONF, data);
}

void BMI088SetGyroRange(gyro_scale_type_t range) {
	if (range == RANGE_2000) {
		gyroRange = 2000;
	} else if (range == RANGE_1000) {
		gyroRange = 1000;
	} else if (range == RANGE_500) {
		gyroRange = 500;
	} else if (range == RANGE_250) {
		gyroRange = 250;
	} else if (range == RANGE_125) {
		gyroRange = 125;
	}

	write8(BMI088_GYRO_I2C_ADDR, BMI088_GYRO_RANGE, (uint8_t)range);
}

void BMI088SetGyroRate(gyro_odr_type_t odr) {
	write8(BMI088_GYRO_I2C_ADDR, BMI088_GYRO_BAND_WIDTH, (uint8_t)odr);
}

void BMI088GetAccel(float* x, float* y, float* z) {
	uint8_t buf[6] = {0};
	uint16_t ax = 0, ay = 0, az = 0;
	float value = 0;

	read(BMI088_ACC_I2C_ADDR, BMI088_ACC_X_LSB, buf, 6);

	ax = buf[0] | (buf[1] << 8);
	ay = buf[2] | (buf[3] << 8);
	az = buf[4] | (buf[5] << 8);

	value = (int16_t)ax;
	*x = accRange * value / 32768;

	value = (int16_t)ay;
	*y = accRange * value / 32768;

	value = (int16_t)az;
	*z = accRange * value / 32768;
}

float BMI088GetAccelX(void) {
	uint16_t ax = 0;
	float value = 0;

	ax = read16(BMI088_ACC_I2C_ADDR, BMI088_ACC_X_LSB);

	value = (int16_t)ax;
	value = accRange * value / 32768;

	return value;
}

float BMI088GetAccelY(void) {
	uint16_t ay = 0;
	float value = 0;

	ay = read16(BMI088_ACC_I2C_ADDR, BMI088_ACC_Y_LSB);

	value = (int16_t)ay;
	value = accRange * value / 32768;

	return value;
}

float BMI088GetAccelZ(void) {
	uint16_t az = 0;
	float value = 0;

	az = read16(BMI088_ACC_I2C_ADDR, BMI088_ACC_Z_LSB);

	value = (int16_t)az;
	value = accRange * value / 32768;

	return value;
}

void BMI088GetGyro(float* x, float* y, float* z) {
	uint8_t buf[6] = {0};
	uint16_t gx = 0, gy = 0, gz = 0;
	float value = 0;

	read(BMI088_GYRO_I2C_ADDR, BMI088_GYRO_RATE_X_LSB, buf, 6);

	gx = buf[0] | (buf[1] << 8);
	gy = buf[2] | (buf[3] << 8);
	gz = buf[4] | (buf[5] << 8);

	value = (int16_t)gx;
	*x = gyroRange * value / 32768;

	value = (int16_t)gy;
	*y = gyroRange * value / 32768;

	value = (int16_t)gz;
	*z = gyroRange * value / 32768;
}

float BMI088GetGyroX(void) {
	uint16_t gx = 0;
	float value = 0;

	gx = read16(BMI088_GYRO_I2C_ADDR, BMI088_GYRO_RATE_X_LSB);

	value = (int16_t)gx;
	value = gyroRange * value / 32768;

	return value;
}

float BMI088GetGyroY(void) {
	uint16_t gy = 0;
	float value = 0;

	gy = read16(BMI088_GYRO_I2C_ADDR, BMI088_GYRO_RATE_Y_LSB);

	value = (int16_t)gy;
	value = gyroRange * value / 32768;

	return value;
}

float BMI088GetGyroZ(void) {
	uint16_t gz = 0;
	float value = 0;

	gz = read16(BMI088_GYRO_I2C_ADDR, BMI088_GYRO_RATE_Z_LSB);

	value = (int16_t)gz;
	value = gyroRange * value / 32768;

	return value;
}

int16_t BMI088GetTemp(void) {
	uint16_t data = 0;

	data = read16Be(BMI088_ACC_I2C_ADDR, BMI088_ACC_TEMP_MSB);
	data = data >> 5;

	if (data > 1023) {
		data = data - 2048;
	}

	return (int16_t)(data / 8 + 23);
}
