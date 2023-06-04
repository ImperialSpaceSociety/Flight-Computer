
#include "sx1272_hal.h"
#include "sx1272.h"

#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

void sx_gpio_write(sx_gpio_t *gpio, bool value) {
	HAL_GPIO_WritePin(gpio->port, gpio->pin, value);
}

int sx_spi_write(uint8_t *buf, uint16_t size, void *spi) {
	return HAL_SPI_Transmit((SPI_HandleTypeDef *) spi, buf, size, 1000);
}

int sx_spi_read(uint8_t *buf, uint16_t size, void *spi) {
	return HAL_SPI_Receive((SPI_HandleTypeDef *) spi, buf, size, 1000);
}

void sx_sleep(uint32_t ms) {
	HAL_Delay(ms);
}

void sx_dlog(const char *format, ...) {
  va_list args;
  va_start(args, format);

  char msg[128] = "sx1272: ";
  strlcat(msg, format, 128);
  strlcat(msg, "\r\n", 128);
  vprintf(msg, args);

  va_end(args);
}

