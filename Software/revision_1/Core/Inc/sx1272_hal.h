
/*
    Lightweight hardware abstraction layer for
    platform dependent functions
 */

#ifndef _sx_hal_h
#define _sx_hal_h

#include <stdint.h>
#include <stdbool.h>


#include "stm32f446xx.h"


typedef struct sx_gpio {

  // Platform-dependent GPIO struct
  // i.e. information needed to write to a GPIO

  // On STM32 using HAL, this contains a port and a pin.

  GPIO_TypeDef *port;
  uint16_t pin;

} sx_gpio_t;



void sx_gpio_write(sx_gpio_t *gpio, int value);

int sx_spi_write(uint8_t *buf, uint16_t size, void *spi);
int sx_spi_read(uint8_t *buf, uint16_t size, void *spi);

void sx_sleep(uint32_t ms);

// Debug logging function
void sx_dlog(const char *format, ...);

#endif
