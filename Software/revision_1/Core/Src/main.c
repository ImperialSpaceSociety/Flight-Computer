/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "usbd_cdc_if.h"
#include "sx1272.h"
#include "L80M39.h"
#include "bme280.h"
#include "BMI088.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_LENGTH 600
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;

/* USER CODE BEGIN PV */
L80M39_t gps;
uint8_t UART1_rxBuffer[BUFFER_LENGTH] = {0};
sx1272_t sx;
sx_gpio_t sx_cs_gpio, sx_tx_gpio, sx_rx_gpio;
uint8_t buf[256] = "Hello, world!";
float temperature;
float humidity;
float pressure;

struct bme280_dev dev_bme280;
struct bme280_data comp_data_bme280;
int8_t rslt_bme280;

char hum_string[50];
char temp_string[50];
char press_string[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	L80M39_parse(&gps, UART1_rxBuffer, BUFFER_LENGTH);
	char test_message[50];
	//sprintf(test_message, "dat: %.2f lat: %.8f lon: %.8f", gps.datetime, gps.latitude, gps.longitude);
	//CDC_Transmit_FS((uint8_t*) test_message, strlen(test_message));
	HAL_UART_Receive_DMA(&huart5, &UART1_rxBuffer[0], BUFFER_LENGTH);
}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim == &htim6)
//    {
//    	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
//    }
//}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_QUADSPI_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_UART5_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* BME280 init */
    dev_bme280.dev_id = BME280_I2C_ADDR_PRIM;
    dev_bme280.intf = BME280_I2C_INTF;
    dev_bme280.read = user_i2c_read;
    dev_bme280.write = user_i2c_write;
    dev_bme280.delay_ms = user_delay_ms;

    rslt_bme280 = bme280_init(&dev_bme280);

    /* BME280 settings */
    dev_bme280.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev_bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev_bme280.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev_bme280.settings.filter = BME280_FILTER_COEFF_16;
    rslt_bme280 = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev_bme280);

    /* Radio INIT */
	//HAL_GPIO_WritePin(Radio_Enable_GPIO_Port, Radio_Enable_Pin, SET);

	sx_cs_gpio = (sx_gpio_t) {
		.port = Radio_Enable_GPIO_Port,
		.pin = Radio_Enable_Pin,
	};

	sx_tx_gpio = (sx_gpio_t) {
		.port = Radio_TX_GPIO_Port,
		.pin = Radio_TX_Pin,
	};

	sx_rx_gpio = (sx_gpio_t) {
		.port = Radio_RX_GPIO_Port,
		.pin = Radio_RX_Pin,
	};

	// Init device structure

	sx = (sx1272_t) {
			.spi = &hspi1,
			.cs_gpio = &sx_cs_gpio,
			.tx_gpio = &sx_tx_gpio,
			.rx_gpio = &sx_rx_gpio,
			.packet_length = 16,
	};

	/*printf("tx (1), rx (2), tx_no_crc (3), echo_src (4), echo_wall (5) ? ");
	fflush(stdout);*/
	int status = sx1272_common_init(&sx, false);

	/* GPS Init */
	HAL_GPIO_WritePin(GPS_Reset_GPIO_Port, GPS_Reset_Pin, 1);
	L80M39_init(&gps);
	HAL_UART_Receive_DMA(&huart5, &UART1_rxBuffer[0], BUFFER_LENGTH);

	/* BMI088 Init */
    if (BMI088Check()) {
    	BMI088SetAccRange(RANGE_3G);
        BMI088Init();
    } else {
        // Error handling
    }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // counter is used for delaying the start of buzzer
  unsigned int counter = 0;

  // buffer for sending messages via USB conn
  char test_message[50];

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* BME280 data acquisition */
	  rslt_bme280 = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev_bme280);
	  dev_bme280.delay_ms(40);
	  rslt_bme280 = bme280_get_sensor_data(BME280_ALL, &comp_data_bme280, &dev_bme280);
	  if(rslt_bme280 == BME280_OK)
	  {
		temperature = comp_data_bme280.temperature / 100.0;
		humidity = comp_data_bme280.humidity / 1024.0;
		pressure = comp_data_bme280.pressure / 10000.0;
	  } else {
		  // Replace with proper error handling
//		  if (HAL_UART_Transmit(&huart2, "Error ", 6, 100)) {
//			  Error_Handler();
//			  HAL_Delay(500);
//		  }
	  }

      /* BMI088 data acquisition */
      float ax = 0, ay = 0, az = 0;
      float gx = 0, gy = 0, gz = 0;
      int16_t temp = 0;

      BMI088GetAccel(&ax, &ay, &az);
      BMI088GetGyro(&gx, &gy, &gz);
      temp = BMI088GetTemp();

      // Conversion of data variable pointers into unsigned int for transmission
	  unsigned int latitudeInt = *(unsigned int*)&gps.latitude;
	  unsigned int longitudeInt = *(unsigned int*)&gps.longitude;
	  unsigned int dateInt = *(unsigned int*)&gps.datetime;
	  unsigned int tempInt = *(unsigned int*)&temperature;
	  unsigned int humidityInt = *(unsigned int*)&humidity;
	  unsigned int pressureInt = *(unsigned int*)&pressure;
	  unsigned int axInt = *(unsigned int*)&ax;
	  unsigned int ayInt = *(unsigned int*)&ay;
	  unsigned int azInt = *(unsigned int*)&az;
	  unsigned int gxInt = *(unsigned int*)&gx;
	  unsigned int gyInt = *(unsigned int*)&gy;
	  unsigned int gzInt = *(unsigned int*)&gz;

	  // data has to be split into 2 batches for transmission
	  // because buffer sent to SX1272 cannot  be more than 16 elements long

	  buf[0] = 0x10;
	  buf[1] = 0x00;
	  buf[2] = 0x00;
	  buf[3] = 0x00;
	  buf[4] = latitudeInt & 0xff;
	  buf[5] = (latitudeInt >> 8) & 0xff;
	  buf[6] = (latitudeInt >> 16) & 0xff;
	  buf[7] = (latitudeInt >> 24) & 0xff;
	  buf[8] = longitudeInt & 0xff;
	  buf[9] = (longitudeInt >> 8) & 0xff;
	  buf[10] = (longitudeInt >> 16) & 0xff;
	  buf[11] = (longitudeInt >> 24) & 0xff;
	  buf[12] = dateInt & 0xff;
	  buf[13] = (dateInt >> 8) & 0xff;
	  buf[14] = (dateInt >> 16) & 0xff;
	  buf[15] = (dateInt >> 24) & 0xff;
	  int ret = sx1272_transmit(&sx, buf);

	HAL_Delay(50);

	  buf[0] = 0x20;
	  buf[1] = 0x00;
	  buf[2] = 0x00;
	  buf[3] = 0x00;
	  buf[4] = tempInt & 0xff;
	  buf[5] = (tempInt >> 8) & 0xff;
	  buf[6] = (tempInt >> 16) & 0xff;
	  buf[7] = (tempInt >> 24) & 0xff;
	  buf[8] = humidityInt & 0xff;
	  buf[9] = (humidityInt >> 8) & 0xff;
	  buf[10] = (humidityInt >> 16) & 0xff;
	  buf[11] = (humidityInt >> 24) & 0xff;
	  buf[12] = pressureInt & 0xff;
	  buf[13] = (pressureInt >> 8) & 0xff;
	  buf[14] = (pressureInt >> 16) & 0xff;
	  buf[15] = (pressureInt >> 24) & 0xff;
	  ret = sx1272_transmit(&sx, buf);

	HAL_Delay(50);

	  buf[0] = 0x30;
	  buf[1] = 0x00;
	  buf[2] = 0x00;
	  buf[3] = 0x00;
	  buf[4] = axInt & 0xff;
	  buf[5] = (axInt >> 8) & 0xff;
	  buf[6] = (axInt >> 16) & 0xff;
	  buf[7] = (axInt >> 24) & 0xff;
	  buf[8] = ayInt & 0xff;
	  buf[9] = (ayInt >> 8) & 0xff;
	  buf[10] = (ayInt >> 16) & 0xff;
	  buf[11] = (ayInt >> 24) & 0xff;
	  buf[12] = azInt & 0xff;
	  buf[13] = (azInt >> 8) & 0xff;
	  buf[14] = (azInt >> 16) & 0xff;
	  buf[15] = (azInt >> 24) & 0xff;
	  ret = sx1272_transmit(&sx, buf);

	HAL_Delay(50);

	  buf[0] = 0x40;
	  buf[1] = 0x00;
	  buf[2] = 0x00;
	  buf[3] = 0x00;
	  buf[4] = gxInt & 0xff;
	  buf[5] = (gxInt >> 8) & 0xff;
	  buf[6] = (gxInt >> 16) & 0xff;
	  buf[7] = (gxInt >> 24) & 0xff;
	  buf[8] = gyInt & 0xff;
	  buf[9] = (gyInt >> 8) & 0xff;
	  buf[10] = (gyInt >> 16) & 0xff;
	  buf[11] = (gyInt >> 24) & 0xff;
	  buf[12] = gzInt & 0xff;
	  buf[13] = (gzInt >> 8) & 0xff;
	  buf[14] = (gzInt >> 16) & 0xff;
	  buf[15] = (gzInt >> 24) & 0xff;
	  ret = sx1272_transmit(&sx, buf);

	  //Printing to USB for debugging
//	sprintf(test_message, "Tx status: %d\r\n", ret);
//	CDC_Transmit_FS((uint8_t*) test_message, strlen(test_message));
//	sprintf(test_message, "Datetime: %x\n\r", dateInt);
//	CDC_Transmit_FS((uint8_t*) test_message, strlen(test_message));
//	sprintf(test_message, "temperature: %4.2f, humidity: %4.2f, pressure: %4.2f\r\n", temperature, humidity, pressure);
//	CDC_Transmit_FS((uint8_t*) test_message, strlen(test_message));
//	sprintf(test_message, "temperature: %x, humidity: %x, pressure: %x\r\n", tempInt, humidityInt, pressureInt);
//	CDC_Transmit_FS((uint8_t*) test_message, strlen(test_message));
//	sprintf(test_message, "Latitude: %x\n\r", latitudeInt);
//	CDC_Transmit_FS((uint8_t*) test_message, strlen(test_message));
//	sprintf(test_message, "Longitude: %x\n\r", longitudeInt);
	sprintf(test_message, "ax: %4.2f, ay: %4.2f, az: %4.2f\r\n", ax, ay, az);
	CDC_Transmit_FS((uint8_t*) test_message, strlen(test_message));
	sprintf(test_message, "gx: %4.2f, gy: %4.2f, gz: %4.2f\r\n", gx, gy, gz);
	CDC_Transmit_FS((uint8_t*) test_message, strlen(test_message));

	HAL_Delay(50);
	counter++;
	// Buzzer will start in 18min = 4500 counts * 240ms / 60 (total HAL delay; it is not very precise)
	if(counter == 4500) {
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 24;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 600;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Radio_RX_GPIO_Port, Radio_RX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Radio_Enable_Pin|GPS_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Radio_Reset_Pin|Indicator_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Acc_Int1_Pin Gyro_Int3_Pin Radio_DIO2_Pin Radio_DIO5_Pin
                           Radio_DIO4_Pin Radio_DIO3_Pin */
  GPIO_InitStruct.Pin = Acc_Int1_Pin|Gyro_Int3_Pin|Radio_DIO2_Pin|Radio_DIO5_Pin
                          |Radio_DIO4_Pin|Radio_DIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Radio_DIO1_Pin Radio_DIO0_Pin Radio_TX_Pin Mag_Data_Ready_Pin */
  GPIO_InitStruct.Pin = Radio_DIO1_Pin|Radio_DIO0_Pin|Radio_TX_Pin|Mag_Data_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Radio_RX_Pin */
  GPIO_InitStruct.Pin = Radio_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Radio_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Radio_Enable_Pin GPS_Reset_Pin */
  GPIO_InitStruct.Pin = Radio_Enable_Pin|GPS_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Radio_Reset_Pin Indicator_LED_Pin */
  GPIO_InitStruct.Pin = Radio_Reset_Pin|Indicator_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
