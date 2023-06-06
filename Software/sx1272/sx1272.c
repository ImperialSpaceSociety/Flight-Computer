
#include "sx1272.h"
#include "sx1272_regs.h"
#include "sx1272_hal.h"
#include <assert.h>

#define BIT(val, bit) (((val) >> (bit)) & 1)

static bool write_reg(sx1272_t *dev, uint8_t reg, uint8_t value) {
  uint8_t data[] = {reg | 0x80, value};
  sx_gpio_write(dev->cs_gpio, 0);
  int ret = sx_spi_write(data, 2, dev->spi);
  sx_gpio_write(dev->cs_gpio, 1);
  sx_sleep(REG_RW_DELAY);
  return ret;
}

static uint8_t read_reg(sx1272_t *dev, uint8_t reg) {
  sx_gpio_write(dev->cs_gpio, 0);
  sx_spi_write(&reg, 1, dev->spi);
  sx_spi_read(&reg, 1, dev->spi);
  sx_gpio_write(dev->cs_gpio, 1);
  sx_sleep(REG_RW_DELAY);
  return reg;
}

int sx1272_common_init(sx1272_t *dev) {

  int ret = 0;

  dev->opDone = false;

  sx_gpio_write(dev->cs_gpio, 1);

  ret = read_reg(dev, SxRegVersion);

  if (ret != 0x22) {
    sx_dlog("error: expected chip ID 0x22, but got 0x%x.", ret);
    return 1;
  }

  write_reg(dev, SxRegOpMode, 0x00);  // sleep mode
  write_reg(dev, SxRegOpMode, 0x80);  // enable lora
  write_reg(dev, SxRegOpMode, 0x81);  // standby mode

  // 125kHz bandwidth, 4/5 coding rate, implicit header, CRC on
  write_reg(dev, SxLoraRegModemConfig1, 0x0e);

  write_reg(dev, SxLoraRegPayloadLength, dev->packet_length);
  write_reg(dev, SxLoraRegMaxPayloadLength, dev->packet_length);

  // Mask all IRQs except RxTimeout, RxDone, PayloadCrcError, TxDone
  write_reg(dev, SxLoraRegIrqFlagsMask, 0x17);

  // Clear all IRQ flags
  write_reg(dev, SxLoraRegIrqFlags, 0xff);

  return 0;
}

int sx1272_transmit(sx1272_t *dev, uint8_t *buf) {

  // Set Dio0 to active high if TxDone
  // (Not in use for now)
  //write_reg(dev, SxRegDioMapping1, 0x40);

  // Assuming RegFifoTxBaseAddr = 0x80, which is the default.
  write_reg(dev, SxLoraRegFifoAddrPtr, 0x80);

  // Fill tx buffer

  uint8_t fifoAddr = SxRegFifo | 0x80;

  sx_gpio_write(dev->cs_gpio, 0);
  sx_spi_write(&fifoAddr, 1, dev->spi);
  sx_spi_write(buf, dev->packet_length, dev->spi);
  sx_gpio_write(dev->cs_gpio, 1);

  // Request TX mode
  write_reg(dev, SxRegOpMode, 0x83);

  // Busy wait until TxDone
  while ( ! BIT(read_reg(dev, SxLoraRegIrqFlags), 3) ) {
    sx_sleep(BUSY_WAIT_DELAY);
  }
  // Clear all IRQ flags
  write_reg(dev, SxLoraRegIrqFlags, 0xff);

  return 0;
}

int sx1272_receive_single(sx1272_t *dev, uint8_t *buf) {

  // Set Dio0 to active high if RxDone
  // (Not in use for now)
  //write_reg(dev, SxRegDioMapping1, 0x00);

  // Assuming RegFifoRxBaseAddr = 0x00, which is the default.
  write_reg(dev, SxLoraRegFifoAddrPtr, 0x00);

  // Request RXSINGLE mode
  write_reg(dev, SxRegOpMode, 0x86);

  // Busy wait until RxDone or RxTimeout
  // RxTimeout should be about 100 seconds?
  int flags = read_reg(dev, SxLoraRegIrqFlags);
  while ( ! BIT(flags, 6) &&	// RxDone
      ! BIT(flags, 7)			// RxTimeout
  ) {
    sx_sleep(BUSY_WAIT_DELAY);
    flags = read_reg(dev, SxLoraRegIrqFlags);
  }

  flags = read_reg(dev, SxLoraRegIrqFlags);
  bool timeout = BIT(flags, 7);
  bool validCrc = BIT(flags, 5);

  if (timeout) {
    return SX_TIMEOUT;
  }

  // Clear all IRQ flags
  write_reg(dev, SxLoraRegIrqFlags, 0xff);

  // Assuming RegFifoRxBaseAddr = 0x00, which is the default.
  write_reg(dev, SxLoraRegFifoAddrPtr, 0x00);

  // Read data from FIFO

  uint8_t fifoAddr = SxRegFifo;

  sx_gpio_write(dev->cs_gpio, 0);
  sx_spi_write(&fifoAddr, 1, dev->spi);
  sx_spi_read(buf, dev->packet_length, dev->spi);
  sx_gpio_write(dev->cs_gpio, 1);


  if (validCrc) {
    return SX_OK;
  }

  return SX_BAD_CRC;
}

int sx1272_current_rssi(sx1272_t *dev) {
  return (int) read_reg(dev, SxLoraRegRssiValue) - 139;
}

int sx1272_packet_rssi(sx1272_t *dev, int pkt_snr) {
  int val = (int) read_reg(dev, SxLoraRegPktRssiValue) - 139;
  if (pkt_snr < 0) {
    val += pkt_snr / 4;
  }
  return val;
}

int sx1272_packet_snr(sx1272_t *dev) {
  uint8_t val = read_reg(dev, SxLoraRegPktSnrValue);
  return ((int8_t) val) / 4;
}

void sx1272_interrupt_handler(sx1272_t *dev) {
  dev->opDone = true;
}



