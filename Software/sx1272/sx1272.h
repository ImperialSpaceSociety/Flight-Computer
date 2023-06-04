

#ifndef _sx1272_h
#define _sx1272_h

#include <stdint.h>
#include <stdbool.h>

#include "sx1272_hal.h"

// 5ms delay between each r/w to a register
// This is not specified by the datasheet, but
// we do it for good measure.
#define REG_RW_DELAY 5

#define BUSY_WAIT_DELAY 50

enum sx_status {
  SX_OK,
  SX_BUSY,
  SX_TIMEOUT,
  SX_BAD_CRC,
  SX_OTHER_ERROR,
};

struct sx1272 {

  // Pointer to platform-dependent SPI device entity
  // e.g. for STM32 HAL, this would point to a SPI_HandleTypeDef.
  // Assumes the SPI device in question has been initialized.
  void *spi;

  // GPIO for SPI chip select pin
  sx_gpio_t *cs_gpio;

  uint16_t packet_length;

  // Interrupt flags
  bool opDone;
};


typedef struct sx1272 sx1272_t;


int sx1272_common_init(sx1272_t *dev);
/*
int sx1272_tx_init(sx1272_t *dev);
int sx1272_rx_init(sx1272_t *dev);
 */

int sx1272_current_rssi(sx1272_t *dev);
int sx1272_packet_rssi(sx1272_t *dev, int pkt_snr);
int sx1272_packet_snr(sx1272_t *dev);

int sx1272_transmit(sx1272_t *dev, uint8_t *buf);
int sx1272_receive_single(sx1272_t *dev, uint8_t *buf);

void sx1272_interrupt_handler(sx1272_t *dev);

#endif
