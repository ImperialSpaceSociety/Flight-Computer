/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Acc_Int1_Pin GPIO_PIN_14
#define Acc_Int1_GPIO_Port GPIOC
#define Gyro_Int3_Pin GPIO_PIN_15
#define Gyro_Int3_GPIO_Port GPIOC
#define Radio_DIO2_Pin GPIO_PIN_1
#define Radio_DIO2_GPIO_Port GPIOC
#define Radio_DIO1_Pin GPIO_PIN_0
#define Radio_DIO1_GPIO_Port GPIOA
#define Radio_DIO0_Pin GPIO_PIN_1
#define Radio_DIO0_GPIO_Port GPIOA
#define Radio_RX_Pin GPIO_PIN_2
#define Radio_RX_GPIO_Port GPIOA
#define Radio_TX_Pin GPIO_PIN_4
#define Radio_TX_GPIO_Port GPIOA
#define Radio_SPI1_SCK_Pin GPIO_PIN_5
#define Radio_SPI1_SCK_GPIO_Port GPIOA
#define Radio_SPI1_MISO_Pin GPIO_PIN_6
#define Radio_SPI1_MISO_GPIO_Port GPIOA
#define Radio_SPI1_MOSI_Pin GPIO_PIN_7
#define Radio_SPI1_MOSI_GPIO_Port GPIOA
#define Battery_Voltage_Pin GPIO_PIN_4
#define Battery_Voltage_GPIO_Port GPIOC
#define Radio_Enable_Pin GPIO_PIN_5
#define Radio_Enable_GPIO_Port GPIOC
#define Flash_QUADSPI_CLK_Pin GPIO_PIN_2
#define Flash_QUADSPI_CLK_GPIO_Port GPIOB
#define Radio_Reset_Pin GPIO_PIN_12
#define Radio_Reset_GPIO_Port GPIOB
#define Radio_DIO5_Pin GPIO_PIN_6
#define Radio_DIO5_GPIO_Port GPIOC
#define Radio_DIO4_Pin GPIO_PIN_7
#define Radio_DIO4_GPIO_Port GPIOC
#define Radio_DIO3_Pin GPIO_PIN_8
#define Radio_DIO3_GPIO_Port GPIOC
#define Flash_QUADSPI_BK1_IO0_Pin GPIO_PIN_9
#define Flash_QUADSPI_BK1_IO0_GPIO_Port GPIOC
#define Mag_Data_Ready_Pin GPIO_PIN_8
#define Mag_Data_Ready_GPIO_Port GPIOA
#define Flash_QUADSPI_BK1_IO1_Pin GPIO_PIN_10
#define Flash_QUADSPI_BK1_IO1_GPIO_Port GPIOC
#define GPS_Reset_Pin GPIO_PIN_11
#define GPS_Reset_GPIO_Port GPIOC
#define GPS_UART_TX_Pin GPIO_PIN_12
#define GPS_UART_TX_GPIO_Port GPIOC
#define GPS_UART_RX_Pin GPIO_PIN_2
#define GPS_UART_RX_GPIO_Port GPIOD
#define Flash_QUADSPI_BK1_NCS_Pin GPIO_PIN_6
#define Flash_QUADSPI_BK1_NCS_GPIO_Port GPIOB
#define Indicator_LED_Pin GPIO_PIN_9
#define Indicator_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
