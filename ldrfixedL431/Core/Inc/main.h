/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ADDR_SCB_VTOR 0xE000ED08
#define ADDR_FLASH_SIZE 0x1FFF75E0
#define ADDR_UNIQUE_ID  0x1FFF7590
#define SYSCLOCKFREQ 16000000  

#define BEGIN_FLASH 0x08008000
#define APP_JMPADDR 0x08008004

#define APPERR_APP_ENTRY_OOR (1 << 0) // App out-of-range address: entry
#define APPERR_APP_CRC_ADDR  (1 << 1) // App out-of-range address: crc
#define APPERR_APP_CHK_ADDR  (1 << 2) // App out-of-range address: checksum
#define APPERR_APP_CRC_NE    (1 << 2) // Computed not equal specified: CRC-32
#define APPERR_APP_CHK_NE    (1 << 2) // Computed not equal specified: checksum



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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_GRN_Pin GPIO_PIN_0
#define LED_GRN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
