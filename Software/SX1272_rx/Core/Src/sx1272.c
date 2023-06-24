#include "sx1272.h"
#include "sx1272_regs.h"
#include "sx1272_hal.h"

#include <string.h>
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

int sx1272_common_init(sx1272_t *dev, bool crcOn) {

  int ret = 0;

  dev->opDone = false;

  sx_gpio_write(dev->cs_gpio, 1);
  sx_gpio_write(dev->tx_gpio, 0);
  sx_gpio_write(dev->rx_gpio, 0);

  ret = read_reg(dev, SxRegVersion);

  if (ret != 0x22) {
    sx_dlog("error: expected chip ID 0x22, but got 0x%x.", ret);
    return 1;
  }

  write_reg(dev, SxRegOpMode, 0x00);  // sleep mode
  write_reg(dev, SxRegOpMode, 0x80);  // enable lora
  write_reg(dev, SxRegOpMode, 0x81);  // standby mode

  // 125kHz bandwidth, 4/5 coding rate, implicit header, CRC on
  write_reg(dev, SxLoraRegModemConfig1, 0x0c | (crcOn << 1));

  // Set rx timeout multiplier to 1023

  // Timeout = multiplier * 2^(Spreading Factor) / (Bandwidth)
  //         = 1023 * 2^7 / 125000 = 1.05s

  uint8_t tmp = read_reg(dev, SxLoraRegModemConfig2);
  tmp = tmp | 0x03;   // Set multiplier MSB
  write_reg(dev, SxLoraRegModemConfig2, tmp);
  write_reg(dev, SxLoraRegSymbTimeoutLsb, 0xff);  // Set multiplier LSB

  write_reg(dev, SxLoraRegPayloadLength, dev->packet_length);
  write_reg(dev, SxLoraRegMaxPayloadLength, dev->packet_length);

  // Mask all IRQs except RxTimeout, RxDone, PayloadCrcError, TxDone
  write_reg(dev, SxLoraRegIrqFlagsMask, 0x17);

  // Clear all IRQ flags
  write_reg(dev, SxLoraRegIrqFlags, 0xff);

  // Clear FIFO
  uint8_t fifoAddr = SxRegFifo | 0x80;
  uint8_t dummy[256];
  memset(dummy, 0, 256);

  sx_gpio_write(dev->cs_gpio, 0);
  sx_spi_write(&fifoAddr, 1, dev->spi);
  sx_spi_write(dummy, 256, dev->spi);
  sx_gpio_write(dev->cs_gpio, 1);

  return 0;
}

sx_status_t sx1272_transmit(sx1272_t *dev, uint8_t *buf) {

  // Set Dio0 to active high if TxDone
  // (Not in use for now)
  //write_reg(dev, SxRegDioMapping1, 0x40);

  sx_gpio_write(dev->tx_gpio, 1);

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

  sx_gpio_write(dev->tx_gpio, 0);

  return SX_OK;
}

sx_status_t sx1272_receive_single(sx1272_t *dev, uint8_t *buf) {

  int ret = SX_OK;

  // Set Dio0 to active high if RxDone
  // (Not in use for now)
  //write_reg(dev, SxRegDioMapping1, 0x00);

  sx_gpio_write(dev->rx_gpio, 1);

  // Assuming RegFifoRxBaseAddr = 0x00, which is the default.
  write_reg(dev, SxLoraRegFifoAddrPtr, 0x00);

  // Request RXSINGLE mode
  write_reg(dev, SxRegOpMode, 0x86);

  // Busy wait until RxDone or RxTimeout
  int flags = read_reg(dev, SxLoraRegIrqFlags);
  while ( ! BIT(flags, 6) &&	// RxDone
          ! BIT(flags, 7)			// RxTimeout
  ) {
    sx_sleep(BUSY_WAIT_DELAY);
    flags = read_reg(dev, SxLoraRegIrqFlags);
  }

  flags = read_reg(dev, SxLoraRegIrqFlags);
  bool timeout = BIT(flags, 7);
  bool badCrc = BIT(flags, 5);

  // Clear all IRQ flags
  write_reg(dev, SxLoraRegIrqFlags, 0xff);

  if (timeout) {
    ret = SX_TIMEOUT;
    goto early_exit;
  }

  // Assuming RegFifoRxBaseAddr = 0x00, which is the default.
  write_reg(dev, SxLoraRegFifoAddrPtr, 0x00);

  // Read data from FIFO

  uint8_t fifoAddr = SxRegFifo;

  sx_gpio_write(dev->cs_gpio, 0);
  sx_spi_write(&fifoAddr, 1, dev->spi);
  sx_spi_read(buf, dev->packet_length, dev->spi);
  sx_gpio_write(dev->cs_gpio, 1);

  if (badCrc) {
    ret = SX_BAD_CRC;
    goto early_exit;
  }

early_exit:
  sx_gpio_write(dev->rx_gpio, 0);
  return ret;
}

sx_status_t sx1272_receive_continuous(sx1272_t *dev, uint8_t *buf, bool denyBadCrc) {

  int ret = SX_OK;

  // Set Dio0 to active high if RxDone
  // (Not in use for now)
  //write_reg(dev, SxRegDioMapping1, 0x00);

  sx_gpio_write(dev->rx_gpio, 1);

  // Assuming RegFifoRxBaseAddr = 0x00, which is the default.
  write_reg(dev, SxLoraRegFifoAddrPtr, 0x00);

  // Request RXCONTINUOUS mode
  write_reg(dev, SxRegOpMode, 0x85);

  // Keep on receiving if CRC is incorrect
  bool badCrc = false;
  do {

    // Busy wait until RxDone
    int flags = read_reg(dev, SxLoraRegIrqFlags);
    while ( ! BIT(flags, 6) ) {
      sx_sleep(BUSY_WAIT_DELAY);
      flags = read_reg(dev, SxLoraRegIrqFlags);
    }

    badCrc = BIT(flags, 5);

    if (badCrc) {
      sx_dlog("Ignoring packet with bad CRC");
    }

    // Clear all IRQ flags
    write_reg(dev, SxLoraRegIrqFlags, 0xff);

    /*
    // Assuming RegFifoRxBaseAddr = 0x00, which is the default.
    write_reg(dev, SxLoraRegFifoAddrPtr, 0x00);
    write_reg(dev, SxLoraRegFifoRxCurrentAddr, 0x00);
    write_reg(dev, SxLoraRegFifoRxByteAddr, 0x00);
    */

  } while (denyBadCrc && badCrc);

  // Standby mode
  write_reg(dev, SxRegOpMode, 0x81);

  // Read data from FIFO

  uint8_t cur = read_reg(dev, SxLoraRegFifoRxCurrentAddr);
  write_reg(dev, SxLoraRegFifoAddrPtr, cur);

  uint8_t fifoAddr = SxRegFifo;

  sx_gpio_write(dev->cs_gpio, 0);
  sx_spi_write(&fifoAddr, 1, dev->spi);
  sx_spi_read(buf, dev->packet_length, dev->spi);
  //sx_spi_read(buf, 256, dev->spi);
  sx_gpio_write(dev->cs_gpio, 1);

  if (badCrc) {
    ret = SX_BAD_CRC;
    goto early_exit;
  }

early_exit:
  sx_gpio_write(dev->rx_gpio, 0);
  return ret;
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
