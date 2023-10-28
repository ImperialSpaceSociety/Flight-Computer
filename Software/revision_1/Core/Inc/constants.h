#ifndef __CONSTANTS__H__
#define __CONSTANTS__H__

// === INCLUDES ===
#include "stm32f4xx_hal.h"

// #define Radio_DIO0_Pin GPIO_PIN_1
// #define Radio_DIO0_GPIO_Port GPIOA
// #define Radio_RX_Pin GPIO_PIN_2
// #define Radio_RX_GPIO_Port GPIOA

// === BUZZER & LED ===
#define BUZ_PIN              GPIO_PIN_3
#define BUZ_PORT             GPIOA
#define LED_PIN              GPIO_PIN_9
#define LED_PORT             GPIOB

// === USB ===
#define USB_DM_PIN           GPIO_PIN_11
#define USB_DP_PIN           GPIO_PIN_12
#define USB_PORT             GPIOA

// === I2C ===
#define I2C_SCL_PIN          GPIO_PIN_7
#define I2C_SDA_PIN          GPIO_PIN_8
#define I2C_PORT             GPIOB

#define BME280_I2C_ADDR      0x76

#define BMI088_ACC_I2C_ADDR  0x18
#define BMI088_GYRO_I2C_ADDR 0x68
#define BMI088_ACC_PIN       GPIO_PIN_14
#define BMI088_GYRO_PIN      GPIO_PIN_15
#define BMI088_PORT          GPIOC



// #define Radio_TX_Pin GPIO_PIN_4
// #define Radio_TX_GPIO_Port GPIOA
// #define Radio_SPI1_SCK_Pin GPIO_PIN_5
// #define Radio_SPI1_SCK_GPIO_Port GPIOA
// #define Radio_SPI1_MISO_Pin GPIO_PIN_6
// #define Radio_SPI1_MISO_GPIO_Port GPIOA
// #define Radio_SPI1_MOSI_Pin GPIO_PIN_7
// #define Radio_SPI1_MOSI_GPIO_Port GPIOA
// #define Battery_Voltage_Pin GPIO_PIN_4
// #define Battery_Voltage_GPIO_Port GPIOC
// #define Radio_Enable_Pin GPIO_PIN_5
// #define Radio_Enable_GPIO_Port GPIOC
// #define Flash_QUADSPI_CLK_Pin GPIO_PIN_2
// #define Flash_QUADSPI_CLK_GPIO_Port GPIOB
// #define Radio_Reset_Pin GPIO_PIN_12
// #define Radio_Reset_GPIO_Port GPIOB
// #define Flash_QUADSPI_BK1_IO0_Pin GPIO_PIN_9
// #define Flash_QUADSPI_BK1_IO0_GPIO_Port GPIOC
// #define Mag_Data_Ready_Pin GPIO_PIN_8
// #define Mag_Data_Ready_GPIO_Port GPIOA
// #define DEBUG_TX_Pin GPIO_PIN_9
// #define DEBUG_TX_GPIO_Port GPIOA
// #define DEBUG_RX_Pin GPIO_PIN_10
// #define DEBUG_RX_GPIO_Port GPIOA
// #define Flash_QUADSPI_BK1_IO1_Pin GPIO_PIN_10
// #define Flash_QUADSPI_BK1_IO1_GPIO_Port GPIOC
// #define GPS_Reset_Pin GPIO_PIN_11
// #define GPS_Reset_GPIO_Port GPIOC
// #define GPS_UART_TX_Pin GPIO_PIN_12
// #define GPS_UART_TX_GPIO_Port GPIOC
// #define GPS_UART_RX_Pin GPIO_PIN_2
// #define GPS_UART_RX_GPIO_Port GPIOD
// #define Flash_QUADSPI_BK1_NCS_Pin GPIO_PIN_6
// #define Flash_QUADSPI_BK1_NCS_GPIO_Port GPIOB

#endif