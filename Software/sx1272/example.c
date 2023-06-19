
/*
 *
 * Demo program of using sx1272 driver
 *     with the LAMBDA-9X module
 *
 */



// STM32 platform specific stuff
// SPI, timer, UART

SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;

// SX1272 device control structure

sx1272_t sx;

// Various control GPIOs: chip select, TX switch, RX switch

sx_gpio_t sx_cs_gpio, sx_tx_gpio, sx_rx_gpio;

// Some control variables

int buttonPressed = 0;
uint8_t buf[256] = "Hello, world!";
int mode = 1;


// Set up external interrupts for button

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	if (pin == BTN1_Pin) {
		if (! HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin)) {
			buttonPressed = 1;
		}
	}
}

int main(void)
{

	// Initialize all STM32 stuff
	
	// ...


	// Set up GPIOs for SX1272 driver

	sx_cs_gpio = (sx_gpio_t) {
		.port = RADIO_CS_GPIO_Port,
		.pin = RADIO_CS_Pin,
	};

	sx_tx_gpio = (sx_gpio_t) {
		.port = RADIO_TXS_GPIO_Port,
		.pin = RADIO_TXS_Pin,
	};

	sx_rx_gpio = (sx_gpio_t) {
		.port = RADIO_RXS_GPIO_Port,
		.pin = RADIO_RXS_Pin,
	};

	// Init device structure
	
	sx = (sx1272_t) {
			.spi = &hspi2,
			.cs_gpio = &sx_cs_gpio,
			.tx_gpio = &sx_tx_gpio,
			.rx_gpio = &sx_rx_gpio,
			.packet_length = 16,
	};

	printf("tx (1), rx (2), tx_no_crc (3), echo_src (4), echo_wall (5) ? ");
	fflush(stdout);

	mode = __io_getchar() - '0';

	printf("\r\nGot mode: %d\r\n", mode);


	if (sx1272_common_init(&sx, !(mode == 3))) {
		Error_Handler();
	}

	printf("sx1272 init success!\r\n");

	if (mode == 3) {
		mode = 1;
		strcpy((char *)buf, "xxxxx, xxxxx!");
	}

	while (1)
	{

		if (mode == 5) {	// echo_wall
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
			uint8_t rcv[16];
			int ret = sx1272_receive_continuous(&sx, rcv, true);

			sx1272_transmit(&sx, rcv);
			printf("Echo: %.16s\r\n", rcv);

			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
		}
		else {
			HAL_Delay(50);
		}

		if (buttonPressed) {
			buttonPressed = 0;

			if (mode == 1) { // TX

				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);

				int ret = sx1272_transmit(&sx, buf);

				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
				printf("Tx status: %d\r\n", ret);

			}
			else if (mode == 2) { // RX

				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);

				htim6.Instance->CNT = 0;
				HAL_TIM_Base_Start(&htim6);

				int ret = sx1272_receive_continuous(&sx, buf, true);

				HAL_TIM_Base_Stop(&htim6);

				printf("Time from last RX: %ldms\r\n", htim6.Instance->CNT / 2);

				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
				printf("RX status: %d\r\n", ret);

				for (int i=0; i<256; i++) {
					if (buf[i] == 0)
						buf[i] = ' ';
				}
				printf("Got: %.256s\r\n", buf);
				memset(buf, 0, 16);
			}
			else if (mode == 4) { // echo_src
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
				sx1272_transmit(&sx, buf);
				printf("Sent message: %.16s\r\n", buf);

				uint8_t rcv[16];
				int ret = sx1272_receive_continuous(&sx, rcv, true);
				printf("Got: %.16s\r\n", rcv);

				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
			}

		}
	}
}

