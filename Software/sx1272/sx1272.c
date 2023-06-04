
#include "sx1272.h"
#include "sx1272_regs.h"
#include "sx1272_hal.h"
#include <assert.h>

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

	return 0;
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


int sx1272_tx_single(sx1272_t *dev, uint8_t *buf) {
	return 0;
}

int sx1272_rx_single(sx1272_t *dev, uint8_t *buf) {
	return 0;
}


